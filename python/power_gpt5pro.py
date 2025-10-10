# example_cuda.py
# MILP Bus Scheduling with NVIDIA cuOpt (linear_programming API)
# Two terminals: CEI (A) and CRBT2 (B), single bus, time-expanded formulation.

from cuopt.linear_programming.problem import Problem, INTEGER, CONTINUOUS, MINIMIZE
from cuopt.linear_programming.solver_settings import SolverSettings

# ---- CONFIG ----
STEP_MIN = 1       # slot size (minutes)
HORIZON_MIN = 200  # planning horizon (minutes)
TAU_MIN = 25       # travel time A<->B (minutes)
CAPACITY = 50      # bus capacity per departure
W_MAX_MIN = 60     # max waiting allowed (minutes)
M = 10_000         # big-M for linking

# PWL for s^2: number of segments over [0, W_MAX_SLOTS]
NUM_PWL_SEG = 6    # e.g., 6 segments => breakpoints every 10 minutes if W_MAX=60

# ---- TIME GRID ----
T = list(range(0, HORIZON_MIN + 1, STEP_MIN))
N = len(T)
TAU_SLOTS = TAU_MIN // STEP_MIN
W_MAX_SLOTS = W_MAX_MIN // STEP_MIN

# Allowed departure times (paper: forbid last tau slots)
if TAU_SLOTS > 0:
    T_dep = T[:-TAU_SLOTS]
else:
    T_dep = T[:]

# ---- PASSENGERS (example) ----
P   = list(range(1, 51))  # CEI -> B
P_r = list(range(1, 3))   # B -> CEI

# Example arrivals (replace with real data)
arr   = {i: T[(2 * i) % N] for i in P}     # CEI arrivals
arr_r = {i: T[(3 * i) % N] for i in P_r}   # B arrivals

def allowed_CEI_times(i):
    if not T_dep:
        return []
    hi = min(arr[i] + W_MAX_MIN, T_dep[-1])
    return [t for t in T_dep if t >= arr[i] and t <= hi]

def allowed_CRBT2_times(j):
    if not T_dep:
        return []
    hi = min(arr_r[j] + W_MAX_MIN, T_dep[-1])
    return [t for t in T_dep if t >= arr_r[j] and t <= hi]

# Safe sum: return None if empty, to avoid boolean constraints
def sum_vars(vlist):
    it = iter(vlist)
    try:
        total = next(it)
    except StopIteration:
        return None
    for v in it:
        total = total + v
    return total

# ---- MODEL ----
prob = Problem("BusScheduling_MILP")

def bin_var(name):
    return prob.addVariable(lb=0, ub=1, vtype=INTEGER, name=name)

def cont_var(lb=0.0, name=""):
    return prob.addVariable(lb=lb, vtype=CONTINUOUS, name=name)

# Decision variables
x   = {(i, t): bin_var(f"x_{i}_{t}")   for i in P   for t in allowed_CEI_times(i)}
x_r = {(j, t): bin_var(f"xr_{j}_{t}")  for j in P_r for t in allowed_CRBT2_times(j)}

ba   = {t: bin_var(f"ba_{t}")   for t in T}  # bus available at CEI
ba_r = {t: bin_var(f"bar_{t}")  for t in T}  # bus available at B
bd   = {t: bin_var(f"bd_{t}")   for t in T}  # depart CEI at t
bd_r = {t: bin_var(f"bdr_{t}")  for t in T}  # depart B at t

# Departure time and waiting (slot units = minutes here)
Dep   = {i: cont_var(0.0, f"Dep_{i}")   for i in P}
Dep_r = {j: cont_var(0.0, f"Depr_{j}")  for j in P_r}
s     = {i: cont_var(0.0, f"s_{i}")     for i in P}
s_r   = {j: cont_var(0.0, f"s_r_{j}")   for j in P_r}

# PWL epigraph variables z ≈ s^2
z   = {i: cont_var(0.0, f"z_{i}")   for i in P}
z_r = {j: cont_var(0.0, f"zr_{j}")  for j in P_r}

# Build PWL segments for f(s)=s^2 on [0, W_MAX_SLOTS]
# Breakpoints: 0, W/N, 2W/N, ..., W
bp = sorted(set(int(round(k * W_MAX_SLOTS / NUM_PWL_SEG)) for k in range(NUM_PWL_SEG + 1)))
if bp[0] != 0:
    bp = [0] + bp
if bp[-1] != W_MAX_SLOTS:
    bp.append(W_MAX_SLOTS)

segments = []
for k in range(len(bp) - 1):
    s0, s1 = bp[k], bp[k + 1]
    if s1 <= s0:
        continue
    # For s^2: slope m = s0 + s1; intercept b = -s0*s1 (secant line)
    m = s0 + s1
    b = -s0 * s1
    segments.append((m, b, s0, s1))

# ---- CONSTRAINTS ----

# F1 / F4: each passenger boards exactly once within allowed times
for i in P:
    times = allowed_CEI_times(i)
    # ถ้า times ว่าง -> โมเดลจะ infeasible (ตาม paper)
    prob.addConstraint(sum_vars([x[(i, t)] for t in times]) == 1, name=f"F1_assign_CEI_{i}")

for j in P_r:
    times = allowed_CRBT2_times(j)
    prob.addConstraint(sum_vars([x_r[(j, t)] for t in times]) == 1, name=f"F4_assign_B_{j}")

# F2/F3 (CEI) and F5/F6 (B)
for t in T_dep:
    # CEI side
    cei_vars = [x[(i, t)] for i in P if (i, t) in x]
    sum_cei = sum_vars(cei_vars)
    if sum_cei is not None:
        # F2: only board if a departure occurs at t
        prob.addConstraint(sum_cei <= M * bd[t], name=f"F2_board_if_depart_A_{t}")
        # F3: capacity
        prob.addConstraint(sum_cei <= CAPACITY,   name=f"F3_capacity_A_{t}")

    # B side
    b_vars = [x_r[(j, t)] for j in P_r if (j, t) in x_r]
    sum_b = sum_vars(b_vars)
    if sum_b is not None:
        # F5: only board if a departure occurs at t (B)
        prob.addConstraint(sum_b <= M * bd_r[t], name=f"F5_board_if_depart_B_{t}")
        # F6: capacity (B)
        prob.addConstraint(sum_b <= CAPACITY,     name=f"F6_capacity_B_{t}")

# F7/F8: depart only if bus available
for t in T:
    prob.addConstraint(bd[t]   <= ba[t],   name=f"F7_depart_if_avail_A_{t}")
    prob.addConstraint(bd_r[t] <= ba_r[t], name=f"F8_depart_if_avail_B_{t}")

# Strengthening: single bus can't depart both sides at the same time
for t in T:
    prob.addConstraint(bd[t] + bd_r[t] <= 1, name=f"X_no_simul_depart_{t}")

# Flow equalities with travel time
for k, t in enumerate(T):
    if k == 0:
        # F15: initial location at exactly one terminal
        prob.addConstraint(ba[t] + ba_r[t] == 1, name="F15_initial_loc")
        continue

    t_prev = T[k - 1]

    # CEI availability propagation
    if k - TAU_SLOTS >= 0:
        t_arr_from_B = T[k - TAU_SLOTS]
        prob.addConstraint(ba[t] == ba[t_prev] - bd[t_prev] + bd_r[t_arr_from_B], name=f"flow_A_{t}")
    else:
        prob.addConstraint(ba[t] == ba[t_prev] - bd[t_prev], name=f"flow_A_{t}")

    # B availability propagation
    if k - TAU_SLOTS >= 0:
        t_arr_from_A = T[k - TAU_SLOTS]
        prob.addConstraint(ba_r[t] == ba_r[t_prev] - bd_r[t_prev] + bd[t_arr_from_A], name=f"flow_B_{t}")
    else:
        prob.addConstraint(ba_r[t] == ba_r[t_prev] - bd_r[t_prev], name=f"flow_B_{t}")

# M3: cannot be available at both terminals simultaneously
for t in T:
    prob.addConstraint(ba[t] + ba_r[t] <= 1, name=f"M3_one_place_{t}")

# T1/T2: link Dep with chosen boarding time
for i in P:
    for t in allowed_CEI_times(i):
        t_slots = t // STEP_MIN
        prob.addConstraint(Dep[i] >= t_slots - M * (1 - x[(i, t)]), name=f"T1_Dep_link_{i}_{t}")

for j in P_r:
    for t in allowed_CRBT2_times(j):
        t_slots = t // STEP_MIN
        prob.addConstraint(Dep_r[j] >= t_slots - M * (1 - x_r[(j, t)]), name=f"T2_Depr_link_{j}_{t}")

# T3/T4: waiting time definition and cap
for i in P:
    arr_slots = arr[i] // STEP_MIN
    prob.addConstraint(s[i] >= Dep[i] - arr_slots, name=f"T3_wait_def_A_{i}")
    prob.addConstraint(s[i] <= W_MAX_SLOTS,        name=f"T3_wait_cap_A_{i}")

for j in P_r:
    arr_r_slots = arr_r[j] // STEP_MIN
    prob.addConstraint(s_r[j] >= Dep_r[j] - arr_r_slots, name=f"T4_wait_def_B_{j}")
    prob.addConstraint(s_r[j] <= W_MAX_SLOTS,            name=f"T4_wait_cap_B_{j}")

# F10/F13: forbid boarding in last tau slots (already removed from T_dep, but double-safety)
if TAU_SLOTS > 0:
    for t in T[-TAU_SLOTS:]:
        for i in P:
            if (i, t) in x:
                prob.addConstraint(x[(i, t)] == 0, name=f"F10_late_forbid_A_{i}_{t}")
        for j in P_r:
            if (j, t) in x_r:
                prob.addConstraint(x_r[(j, t)] == 0, name=f"F13_late_forbid_B_{j}_{t}")

    # Also forbid bus departures too late
    for t in T[-TAU_SLOTS:]:
        prob.addConstraint(bd[t] == 0,   name=f"Z_no_depart_late_A_{t}")
        prob.addConstraint(bd_r[t] == 0, name=f"Z_no_depart_late_B_{t}")

# PWL epigraph for s^2 and s_r^2: z >= m*s + b for every segment
for i in P:
    for idx, (m, b, s0, s1) in enumerate(segments):
        prob.addConstraint(z[i] >= m * s[i] + b, name=f"PWL_A_{i}_{idx}")
for j in P_r:
    for idx, (m, b, s0, s1) in enumerate(segments):
        prob.addConstraint(z_r[j] >= m * s_r[j] + b, name=f"PWL_B_{j}_{idx}")

# ---- OBJECTIVE ----
# Minimize sum of piecewise-linear approximation of squared waits
prob.setObjective(
    sum(z[i] for i in P) + sum(z_r[j] for j in P_r),
    sense=MINIMIZE
)

# ---- SOLVE ----
settings = SolverSettings()
settings.set_parameter("time_limit", 60)  # seconds
# settings.set_parameter("threads", 4)

prob.solve(settings)

print("Status:", prob.Status.name)
print("SolveTime:", getattr(prob, "SolveTime", None))
print("Objective (sum of approx wait^2):", getattr(prob, "ObjValue", None))

# Pretty print solution
if prob.Status.name in ("Optimal", "Feasible"):
    for i in P:
        chosen = [(t, x[(i, t)].getValue()) for t in allowed_CEI_times(i) if x[(i, t)].getValue() > 0.5]
        if chosen:
            t_sel = chosen[0][0]
            print(f"CEI passenger {i}: boards at t={t_sel} min, wait={s[i].getValue()} min, z~={z[i].getValue()}")
    for j in P_r:
        chosen = [(t, x_r[(j, t)].getValue()) for t in allowed_CRBT2_times(j) if x_r[(j, t)].getValue() > 0.5]
        if chosen:
            t_sel = chosen[0][0]
            print(f"B passenger {j}: boards at t={t_sel} min, wait={s_r[j].getValue()} min, z~={z_r[j].getValue()}")

    print("\nBus availability / departures:")
    for t in T:
        if ba[t].getValue() > 0.5:   print(f"  t={t:3d} A-available")
        if ba_r[t].getValue() > 0.5: print(f"  t={t:3d} B-available")
        if bd[t].getValue() > 0.5:   print(f"  t={t:3d} depart A")
        if bd_r[t].getValue() > 0.5: print(f"  t={t:3d} depart B")