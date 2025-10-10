#!/usr/bin/env python3
import json, sys, math, argparse
from collections import defaultdict, Counter

def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def norm_pid(x):
    return str(x)

def index_payload(payload):
    deps = []
    for d in payload.get("departures", []):
        deps.append({"terminal":d["terminal"], "bus":int(d["bus"]), "t":int(d["t"])})
    deps.sort(key=lambda e: (e["t"], e["bus"], e["terminal"]))
    asg = []
    asg_by_event = defaultdict(list)
    for a in payload.get("assignments", []):
        rec = {"terminal":a["terminal"], "p":norm_pid(a["p"]), "bus":int(a["bus"]), "t":int(a["t"])}
        asg.append(rec)
        asg_by_event[(rec["terminal"], rec["bus"], rec["t"])].append(rec["p"])
    arr = {"CEI":{}, "T2":{}}
    for r in payload.get("arrivals",{}).get("CEI",[]): arr["CEI"][norm_pid(r["p"])] = int(r["arr"])
    for r in payload.get("arrivals",{}).get("T2" ,[]): arr["T2" ][norm_pid(r["p"])] = int(r["arr"])
    meta = payload.get("meta",{})
    return {
        "deps": deps,
        "asg": asg,
        "asg_by_event": asg_by_event,
        "arrivals": arr,
        "meta": meta,
        "status": str(payload.get("status","")).upper(),
        "objective": payload.get("objective", None),
        "init": payload.get("initial_positions", [])
    }

def index_expected(exp):
    deps = []
    for d in exp.get("expected_departures", []):
        deps.append({"terminal":d["terminal"], "bus":int(d["bus"]), "t":int(d["t"])})
    deps.sort(key=lambda e: (e["t"], e["bus"], e["terminal"]))
    asg = []
    asg_by_event = defaultdict(list)
    for a in exp.get("expected_assignments", []):
        rec = {"terminal":a["terminal"], "p":norm_pid(a["p"]), "bus":int(a["bus"]), "t":int(a["t"])}
        asg.append(rec)
        asg_by_event[(rec["terminal"], rec["bus"], rec["t"])].append(rec["p"])
    arr = {"CEI":{}, "T2":{}}
    for k,v in exp.get("arrivals",{}).get("CEI",{}).items(): arr["CEI"][norm_pid(k)] = int(v)
    for k,v in exp.get("arrivals",{}).get("T2" ,{}).items(): arr["T2" ][norm_pid(k)] = int(v)
    meta = exp.get("meta",{})
    init = exp.get("init", exp.get("initial_positions", []))
    infeasible = bool(exp.get("infeasible", False))
    obj = exp.get("expected_objective", None)
    return {"deps":deps, "asg":asg, "asg_by_event":asg_by_event, "arrivals":arr, "meta":meta, "init":init, "infeasible":infeasible, "expected_objective":obj}

def build_label_mapping(exp_idx, got_idx):
    mapping = {}   # label -> pid
    inverse = {}   # pid -> label
    errors = []
    keys = set(exp_idx["asg_by_event"].keys()) | set(got_idx["asg_by_event"].keys())
    for key in sorted(keys):
        e_list = sorted(exp_idx["asg_by_event"].get(key, []))
        g_list = sorted(got_idx["asg_by_event"].get(key, []))
        if len(e_list) != len(g_list):
            errors.append(f"[ASSIGN-COUNT] event {key} expected {len(e_list)} got {len(g_list)}")
            continue
        for lab, pid in zip(e_list, g_list):
            if lab in mapping and mapping[lab] != pid:
                errors.append(f"[MAP] label {lab} -> {mapping[lab]} and {pid}")
            mapping[lab] = pid
            if pid in inverse and inverse[pid] != lab:
                errors.append(f"[MAP] pid {pid} -> {inverse[pid]} and {lab}")
            inverse[pid] = lab
    return mapping, inverse, errors

def assert_equal_sets(got, exp, what, errors):
    g = sorted(got, key=lambda e: (e["t"], e["bus"], e["terminal"]))
    e = sorted(exp, key=lambda e: (e["t"], e["bus"], e["terminal"]))
    if g != e:
        errors.append(f"[{what}] mismatch\nexpected={e}\n   got={g}")

def squared_objective(assignments, arrivals):
    s = 0
    for a in assignments:
        term, p, t = a["terminal"], a["p"], a["t"]
        s += (t - arrivals[term][p])**2
    return s

def check_flow(exp_meta, exp_init, deps, errors):
    tau = int(exp_meta["tau"])
    init = {int(d["bus"]): d["terminal"] for d in exp_init} if exp_init else {}
    busy_until = {b: -10**9 for b in init}
    cur_term = dict(init)
    for d in deps:
        b, term, t = int(d["bus"]), d["terminal"], int(d["t"])
        if b not in cur_term:
            cur_term[b] = term
            busy_until[b] = -10**9
        if t < busy_until[b]:
            errors.append(f"[FLOW] bus{b} departs at t={t} while traveling until {busy_until[b]}")
        if term != cur_term[b]:
            errors.append(f"[LOC] bus{b} departs {term} but at {cur_term[b]}")
        busy_until[b] = t + tau
        cur_term[b] = "T2" if term == "CEI" else "CEI"

def check_case(expected_path, result_path):
    exp = load_json(expected_path)
    got = load_json(result_path)

    exp_idx = index_expected(exp)
    got_idx = index_payload(got)
    errors = []

    if exp_idx["infeasible"]:
        st = got_idx["status"]
        if "INF" not in st and "NO" not in st:
            errors.append(f"[STATUS] expected infeasible, got status={st}")
        return errors

    for k in ("tau","capacity","w_max","T_end"):
        if k in exp_idx["meta"] and k in got_idx["meta"]:
            if int(exp_idx["meta"][k]) != int(got_idx["meta"][k]):
                errors.append(f"[META] {k} expected {exp_idx['meta'][k]} got {got_idx['meta'][k]}")

    assert_equal_sets(got_idx["deps"], exp_idx["deps"], "DEPARTURES", errors)

    mapping, inverse, map_errs = build_label_mapping(exp_idx, got_idx)
    errors.extend(map_errs)

    got_asg_labeled = []
    for a in got_idx["asg"]:
        pid = a["p"]
        label = inverse.get(pid, pid)
        got_asg_labeled.append({"terminal":a["terminal"], "p":label, "bus":a["bus"], "t":a["t"]})
    assert_equal_sets(got_asg_labeled, exp_idx["asg"], "ASSIGNMENTS", errors)

    cap = int(exp_idx["meta"]["capacity"])
    seen = {"CEI":Counter(), "T2":Counter()}
    for key, plist in got_idx["asg_by_event"].items():
        if len(plist) > cap:
            term, bus, t = key
            errors.append(f"[CAP] {term} bus{bus} t={t}: {len(plist)}>{cap}")
        term = key[0]
        for p in plist:
            seen[term][p] += 1
    for term in ("CEI","T2"):
        for lab in exp_idx["arrivals"][term].keys():
            pid = mapping.get(lab)
            cnt = seen[term][pid] if pid is not None else 0
            if cnt != 1:
                errors.append(f"[F1/F4] {term}:{lab} assigned {cnt} times")

    T_end = int(exp_idx["meta"]["T_end"]); tau = int(exp_idx["meta"]["tau"]); wmax = int(exp_idx["meta"]["w_max"])
    for a in got_idx["asg"]:
        term, pid, t = a["terminal"], a["p"], int(a["t"])
        if pid not in got_idx["arrivals"][term]:
            errors.append(f"[ARR] missing arrival for {term} pid={pid}")
            continue
        arr = int(got_idx["arrivals"][term][pid])
        wait = t - arr
        if wait < 0:
            errors.append(f"[ARR] negative wait {term} pid={pid} arr={arr} t={t}")
        if wait > wmax:
            errors.append(f"[T3/T4] wait {wait} > w_max {wmax} for {term} pid={pid}")
        if t > T_end - tau:
            errors.append(f"[F10/F13] departure {t} > T_end - tau ({T_end - tau})")

    check_flow(exp_idx["meta"], exp_idx["init"], got_idx["deps"], errors)

    if exp_idx["expected_objective"] is not None:
        exp_obj = int(exp_idx["expected_objective"])
        got_obj = got_idx["objective"]
        if got_obj is None or abs(got_obj - exp_obj) > 1e-6:
            errors.append(f"[OBJ] objective {got_obj} != expected {exp_obj}")
        # Optional internal check using expected arrivals/labels:
        if exp_idx["arrivals"]["CEI"] or exp_idx["arrivals"]["T2"]:
            obj_chk = squared_objective(exp_idx["asg"], exp_idx["arrivals"])
            if abs(obj_chk - exp_obj) > 1e-9:
                errors.append(f"[OBJ-CHECK] expected {exp_obj} but recomputed {obj_chk}")

    return errors

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", required=True, help="Expected-case JSON path")
    ap.add_argument("result", help="Solver output JSON (result.json)")
    args = ap.parse_args()

    errs = check_case(args.case, args.result)
    if errs:
        print("✗ TEST FAILED")
        for e in errs:
            print(" -", e)
        # sys.exit(1)
    else:
        print("✓ TEST PASSED")
        # sys.exit(0)

if __name__ == "__main__":
    main()
