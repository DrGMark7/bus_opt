# bus_scheduling_cuopt.py
# MILP Bus Scheduling with NVIDIA cuOpt (linear_programming API)
# Two terminals: CEI (A) and CRBT2 (B), single bus, time-expanded formulation.

from cuopt.linear_programming.problem import Problem, INTEGER, CONTINUOUS, MINIMIZE
from cuopt.linear_programming.solver_settings import SolverSettings

# ---------------------- CONFIG ----------------------
STEP_MIN = 1                 # slot size (minutes)
HORIZON_MIN = 200            # planning horizon (minutes)
TAU_MIN = 25                 # travel time A<->B (minutes)
CAPACITY = 50                # bus capacity per departure
W_MAX_MIN = 60               # max waiting allowed (minutes)

# Downsampled discrete time grid: 0,5,10,...,HORIZON_MIN
T = list(range(0, HORIZON_MIN + 1, STEP_MIN))
N = len(T)
TAU_SLOTS = TAU_MIN // STEP_MIN
W_MAX_SLOTS = W_MAX_MIN // STEP_MIN

# Passengers
P = list(range(1, 51))       # CEI -> B  (change to your size, e.g. 1..50)
P_r = list(range(1, 3))      # B -> CEI  (e.g. 1..2)

# Example arrivals snapped to the grid (replace with your real data)
# Must be in minutes and a member of T
arr  = {i: T[(2*i) % N] for i in P}     # CEI arrivals
arr_r = {i: T[(3*i) % N] for i in P_r}  # B arrivals

# (Optional) prune: forbid boarding before arrival
def allowed_CEI_times(i):   # times t where passenger i may board at CEI
    return [t for t in T if (t >= arr[i]) and (t <= min(arr[i] + W_MAX_MIN, T[-1]))]

def allowed_CRBT2_times(i): # times t where passenger i may board at B
    return [t for t in T if (t >= arr_r[i]) and (t <= min(arr_r[i] + W_MAX_MIN, T[-1]))]

# ---------------------- MODEL ----------------------
prob = Problem("BusScheduling_MILP")

# Helper to make "binary" vars: use INTEGER with [0,1]
def bin_var(name):
    return prob.addVariable(lb=0, ub=1, vtype=INTEGER, name=name)

def cont_var(lb=0.0, name=""):
    return prob.addVariable(lb=lb, vtype=CONTINUOUS, name=name)

# Decision vars
# x[i,t] : passenger i (CEI) boards at time t
# x_r[j,t]: passenger j (B) boards at time t
x = {(i, t): bin_var(f"x_{i}_{t}") for i in P for t in allowed_CEI_times(i)}
x_r = {(j, t): bin_var(f"xr_{j}_{t}") for j in P_r for t in allowed_CRBT2_times(j)}

# Bus state and movements
ba   = {t: bin_var(f"ba_{t}")   for t in T}  # bus available @ CEI
ba_r = {t: bin_var(f"bar_{t}")  for t in T}  # bus available @ B
bd   = {t: bin_var(f"bd_{t}")   for t in T}  # bus departs CEI at t
bd_r = {t: bin_var(f"bdr_{t}")  for t in T}  # bus departs B at t

# Departure time (slot units) and waiting times (slot units) for objective
Dep   = {i: cont_var(0.0, f"Dep_{i}")   for i in P}
Dep_r = {j: cont_var(0.0, f"Depr_{j}")  for j in P_r}
s     = {i: cont_var(0.0, f"s_{i}")     for i in P}
s_r   = {j: cont_var(0.0, f"s_r_{j}")   for j in P_r}

# ---------------------- CONSTRAINTS ----------------------

# F1: Each CEI passenger boards exactly once (over allowed times)
for i in P:
    prob.addConstraint(
        sum(x[(i, t)] for t in allowed_CEI_times(i)) == 1,
        name=f"F1_assign_CEI_{i}"
    )

# F4: Each B passenger boards exactly once
for j in P_r:
    prob.addConstraint(
        sum(x_r[(j, t)] for t in allowed_CRBT2_times(j)) == 1,
        name=f"F4_assign_B_{j}"
    )

# Capacity link with departures (F2,F3 at CEI; F5,F6 at B)
for t in T:
    prob.addConstraint(
        sum(x[(i, t)] for i in P if (i, t) in x) <= CAPACITY * bd[t],
        name=f"cap_CEI_{t}"
    )
    prob.addConstraint(
        sum(x_r[(j, t)] for j in P_r if (j, t) in x_r) <= CAPACITY * bd_r[t],
        name=f"cap_B_{t}"
    )

# F7,F8: can depart only if bus available at that terminal
for t in T:
    prob.addConstraint(bd[t]   <= ba[t],   name=f"depart_if_avail_A_{t}")
    prob.addConstraint(bd_r[t] <= ba_r[t], name=f"depart_if_avail_B_{t}")

# Flow equalities (F9–F12) with travel time lag tau
# Use time index k to reference previous slot and lagged arrivals
for k, t in enumerate(T):
    if k == 0:
        # F11: initial location: bus starts at exactly one terminal
        prob.addConstraint(ba[t] + ba_r[t] == 1, name="F11_initial_loc")
        continue

    t_prev = T[k - 1]

    # ba[t] = ba[t-1] - bd[t-1] + (arrival from B if k - TAU_SLOTS >= 0)
    if k - TAU_SLOTS >= 0:
        t_arr_from_B = T[k - TAU_SLOTS]
        prob.addConstraint(
            ba[t] == ba[t_prev] - bd[t_prev] + bd_r[t_arr_from_B],
            name=f"flow_A_{t}"
        )
    else:
        prob.addConstraint(
            ba[t] == ba[t_prev] - bd[t_prev],
            name=f"flow_A_{t}"
        )

    # ba_r[t] = ba_r[t-1] - bd_r[t-1] + (arrival from A if feasible)
    if k - TAU_SLOTS >= 0:
        t_arr_from_A = T[k - TAU_SLOTS]
        prob.addConstraint(
            ba_r[t] == ba_r[t_prev] - bd_r[t_prev] + bd[t_arr_from_A],
            name=f"flow_B_{t}"
        )
    else:
        prob.addConstraint(
            ba_r[t] == ba_r[t_prev] - bd_r[t_prev],
            name=f"flow_B_{t}"
        )

# M3: cannot be available at both terminals simultaneously
for t in T:
    prob.addConstraint(ba[t] + ba_r[t] <= 1, name=f"M3_one_place_{t}")

# T1/T2: link Dep with chosen departure slot (Dep in slot units)
# Dep[i] >= t_slots - M*(1 - x[i,t])
M = 10_000
for i in P:
    for t in allowed_CEI_times(i):
        t_slots = t // STEP_MIN
        prob.addConstraint(Dep[i] >= t_slots - M * (1 - x[(i, t)]),
                           name=f"T1_Dep_link_{i}_{t}")

for j in P_r:
    for t in allowed_CRBT2_times(j):
        t_slots = t // STEP_MIN
        prob.addConstraint(Dep_r[j] >= t_slots - M * (1 - x_r[(j, t)]),
                           name=f"T2_Depr_link_{j}_{t}")

# F14/F15: waiting time definition & bound (in slots)
for i in P:
    arr_slots = arr[i] // STEP_MIN
    prob.addConstraint(s[i] >= Dep[i] - arr_slots, name=f"wait_def_A_{i}")
    prob.addConstraint(s[i] <= W_MAX_SLOTS,        name=f"wait_cap_A_{i}")

for j in P_r:
    arr_r_slots = arr_r[j] // STEP_MIN
    prob.addConstraint(s_r[j] >= Dep_r[j] - arr_r_slots, name=f"wait_def_B_{j}")
    prob.addConstraint(s_r[j] <= W_MAX_SLOTS,            name=f"wait_cap_B_{j}")

# Forbid boarding too late near horizon (F10/F13): last tau slots
if TAU_SLOTS > 0:
    last_times = T[-TAU_SLOTS:]
    for t in last_times:
        for i in P:
            if (i, t) in x:
                prob.addConstraint(x[(i, t)] == 0, name=f"late_forbid_A_{i}_{t}")
        for j in P_r:
            if (j, t) in x_r:
                prob.addConstraint(x_r[(j, t)] == 0, name=f"late_forbid_B_{j}_{t}")

# (Already enforced by pruning) — forbid boarding before arrival:
# x[(i,t)] exists only when t >= arr[i]; same for x_r.

# ---------------------- OBJECTIVE ----------------------
# Minimize total waiting time (linear MILP objective)
prob.setObjective(
    sum(s[i] for i in P) + sum(s_r[j] for j in P_r),
    sense=MINIMIZE
)

# ---------------------- SOLVE ----------------------
settings = SolverSettings()
settings.set_parameter("time_limit", 60)      # seconds
# settings.set_parameter("threads", 4)

prob.solve(settings)

print("Status:", prob.Status.name)
print("SolveTime:", getattr(prob, "SolveTime", None))
print("Objective (total wait slots):", getattr(prob, "ObjValue", None))

# Pretty print solution
if prob.Status.name in ("Optimal", "Feasible"):
    # chosen departures (CEI)
    for i in P:
        chosen = [(t, x[(i, t)].getValue()) for t in allowed_CEI_times(i) if x[(i, t)].getValue() > 0.5]
        if chosen:
            t_sel = chosen[0][0]
            print(f"CEI passenger {i}: boards at t={t_sel} min, wait={s[i].getValue()} slots")
    # chosen departures (B)
    for j in P_r:
        chosen = [(t, x_r[(j, t)].getValue()) for t in allowed_CRBT2_times(j) if x_r[(j, t)].getValue() > 0.5]
        if chosen:
            t_sel = chosen[0][0]
            print(f"B passenger {j}: boards at t={t_sel} min, wait={s_r[j].getValue()} slots")

    # bus movements
    print("\nBus availability / departures:")
    for t in T:
        if ba[t].getValue() > 0.5:   print(f"  t={t:3d} A-available")
        if ba_r[t].getValue() > 0.5: print(f"  t={t:3d} B-available")
        if bd[t].getValue() > 0.5:   print(f"  t={t:3d} depart A")
        if bd_r[t].getValue() > 0.5: print(f"  t={t:3d} depart B")
