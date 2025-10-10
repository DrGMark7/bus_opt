#!/usr/bin/env python3
import json, sys, math
from collections import defaultdict, Counter

def load_payload(path):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read().strip()
        if txt.startswith("{"):
            return json.loads(txt)
        else:
            return json.loads(txt.splitlines()[0])

def build_index(payload):
    meta = payload["meta"]
    tau = meta["tau"]
    cap = meta["capacity"]
    wmax = meta["w_max"]
    T_start, T_end = meta["T_start"], meta["T_end"]

    # Arrivals
    arrivals = {"CEI": {}, "T2": {}}
    for rec in payload["arrivals"]["CEI"]:
        arrivals["CEI"][rec["p"]] = rec["arr"]
    for rec in payload["arrivals"]["T2"]:
        arrivals["T2"][rec["p"]] = rec["arr"]

    # Initial positions
    initpos = {}
    for rec in payload["initial_positions"]:
        initpos[rec["bus"]] = rec["terminal"]

    # Departures
    deps = defaultdict(set)  # key=(terminal,bus) -> set of t
    all_dep_times_by_bus = defaultdict(list)  # bus -> [(t, terminal)]
    for rec in payload["departures"]:
        term, bus, t = rec["terminal"], rec["bus"], rec["t"]
        deps[(term, bus)].add(t)
        all_dep_times_by_bus[bus].append((t, term))
    for bus in all_dep_times_by_bus:
        all_dep_times_by_bus[bus].sort()

    # Assignments
    asg = defaultdict(list)  # (terminal,bus,t) -> [p,...]
    per_passenger_count = {"CEI": Counter(), "T2": Counter()}
    stored_asg = []
    for rec in payload["assignments"]:
        term, p, bus, t, wait = rec["terminal"], rec["p"], rec["bus"], rec["t"], rec["wait"]
        asg[(term,bus,t)].append(p)
        per_passenger_count[term][p] += 1
        stored_asg.append((term,p,bus,t,wait))

    return {
        "tau": tau, "cap": cap, "wmax": wmax, "T_end": T_end,
        "arrivals": arrivals, "initpos": initpos,
        "deps": deps, "dep_seq": all_dep_times_by_bus,
        "asg": asg, "asg_list": stored_asg,
        "objective": payload.get("objective"),
        "status": payload.get("status")
    }

def check_unique_assignment(idx, errors):
    # F1/F4: ผู้โดยสารแต่ละคนต้องถูก assign = 1 เที่ยว
    for term in ("CEI","T2"):
        arr = idx["arrivals"][term]
        cnt = idx["asg"]
        seen = Counter()
        for (tm,bus,t), plist in idx["asg"].items():
            if tm != term: continue
            for p in plist: seen[p] += 1
        for p in arr.keys():
            if seen[p] != 1:
                errors.append(f"[F1/F4] Passenger {term}:{p} assigned {seen[p]} times (expected 1)")

def check_assignment_link_to_departure(idx, errors):
    # x ≤ bd: ทุก assignment ต้องมี departure ตรงกัน
    for (term,p,bus,t,wait) in idx["asg_list"]:
        if t not in idx["deps"][(term,bus)]:
            errors.append(f"[Link x→bd] Assignment without matching departure: {term} bus{bus} t={t} p={p}")

def check_capacity(idx, errors):
    # F3/F6: จำนวนผู้โดยสารต่อเที่ยว ≤ capacity
    cap = idx["cap"]
    for key, plist in idx["asg"].items():
        if len(plist) > cap:
            term,bus,t = key
            errors.append(f"[F3/F6] Capacity exceeded at {term} bus{bus} t={t}: {len(plist)} > {cap}")

def check_wait_and_windows(idx, errors):
    # T3/T4 + last-window + wait = t - arr
    tau, wmax, Tend = idx["tau"], idx["wmax"], idx["T_end"]
    for (term,p,bus,t,wait) in idx["asg_list"]:
        arr = idx["arrivals"][term].get(p, None)
        if arr is None:
            errors.append(f"[ARR] Missing arrival for {term}:{p}")
            continue
        if wait != t - arr:
            errors.append(f"[WAIT] Wrong wait for {term}:{p} at t={t}: got {wait}, expect {t - arr}")
        if wait < 0:
            errors.append(f"[ARR] Negative wait for {term}:{p}: arrival {arr}, assign t={t}")
        if wait > wmax:
            errors.append(f"[T3/T4] Wait {wait} > w_max {wmax} for {term}:{p}")
        if t > Tend - tau:
            errors.append(f"[Last-window] Departure at t={t} beyond T_end - tau ({Tend - tau})")

def check_bus_flow(idx, errors):
    # F7–F12, M3: รถคันเดียวออกได้ทีละฝั่ง, ต้องคั่นด้วย tau, ห้ามออกระหว่างกำลังเดินทาง
    tau = idx["tau"]
    initpos = idx["initpos"]
    dep_seq = idx["dep_seq"]

    for bus, seq in dep_seq.items():
        seq_sorted = sorted(seq)  # [(t,term)]
        # หา initial terminal
        cur_term = initpos.get(bus, None)
        cur_free_time = 0  # earliest time the bus can depart (not traveling)

        if cur_term is None and seq_sorted:
            # ถ้าไม่ระบุ initial position แต่มีตาราง อนุโลมจากการออกเที่ยวแรก
            cur_term = seq_sorted[0][1]

        last_t = -10**9
        for (t, term) in seq_sorted:
            # ห้ามออกสองฝั่งเวลาเดียวกัน (จะชนด้วยการเรียงลำดับ)
            if t == last_t:
                errors.append(f"[M3] Bus{bus} multiple departures at same t={t} (check both terminals)")
            last_t = t

            # ต้องอยู่ฝั่งเดียวกับเที่ยวที่ออก และต้องไม่ติดเดินทางอยู่
            if t < cur_free_time:
                errors.append(f"[FLOW] Bus{bus} departs at t={t} while still traveling until {cur_free_time}")
            if term != cur_term:
                errors.append(f"[F7/F8] Bus{bus} departs from {term} at t={t} but current terminal is {cur_term}")

            # หลังออก จะไปอีกฝั่ง และพร้อมอีกทีที่ t+tau
            cur_free_time = t + tau
            cur_term = "T2" if term == "CEI" else "CEI"

def check_objective(idx, errors):
    # ตรวจ sum(wait) = objective (เผื่อ tolerance)
    if idx["objective"] is None:
        return
    s = 0
    for (_,_,_,_,w) in idx["asg_list"]:
        s += w
    if not math.isfinite(idx["objective"]) or abs(idx["objective"] - s) > 1e-6:
        errors.append(f"[OBJ] objective reported {idx['objective']} but recomputed {s}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python evbus_checker.py result.json", file=sys.stderr)
        sys.exit(2)

    payload = load_payload(sys.argv[1])
    idx = build_index(payload)

    errors = []
    # Order checking : Structure → Linking → Time/Capacity Constraints → Bus Flow → Objective
    check_unique_assignment(idx, errors)
    check_assignment_link_to_departure(idx, errors)
    check_capacity(idx, errors)
    check_wait_and_windows(idx, errors)
    check_bus_flow(idx, errors)
    check_objective(idx, errors)

    if errors:
        print("✗ TEST FAILED")
        for e in errors:
            print(" -", e)
        sys.exit(1)
    else:
        print("✓ TEST PASSED")
        print(f"status={payload.get('status')} objective={payload.get('objective')}")
        sys.exit(0)

if __name__ == "__main__":
    main()
