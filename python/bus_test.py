# bus_model_highs_milp.py  — MILP (รองรับ HiGHS)
import pyomo.environ as pyo

# ---------- Time grid ----------
STEP_MIN = 1
T_list   = list(range(0, 200+1, STEP_MIN))     # [0,5,10,...,200]
T        = T_list
N        = len(T_list)

# ---------- Sets ----------
B   = [1]                          # ประเภทบัสเดียว
P   = list(range(1, 51))           # ผู้โดยสารฝั่ง CEI
P_r = list(range(1, 10))            # ผู้โดยสารฝั่ง CRBT2

# ---------- Params ----------
tau_min    = 25
tau_slots  = tau_min // STEP_MIN   # ดีเลย์เป็น "จำนวนช่อง"
c_max      = 50
M          = 10_000
w_max_min  = 60
w_max      = w_max_min // STEP_MIN # หน่วยเป็น "ช่องเวลา"

# ตัวอย่าง arrival ให้อยู่บนกริด
arr   = {i: T_list[(2*i) % N] for i in P}
arr_r = {i: T_list[(3*i) % N] for i in P_r}
arr_s   = {i: arr[i]  / STEP_MIN for i in P}    # แปลงเป็น "ช่องเวลา"
arr_r_s = {i: arr_r[i]/ STEP_MIN for i in P_r}

# ---------- Model ----------
m = pyo.ConcreteModel()

# Vars
m.x    = pyo.Var(P, B, T, domain=pyo.Binary)        # CEI assignment
m.x_r  = pyo.Var(P_r, B, T, domain=pyo.Binary)      # CRBT2 assignment
m.ba   = pyo.Var(T, B, domain=pyo.Binary)           # bus avail @ CEI
m.ba_r = pyo.Var(T, B, domain=pyo.Binary)           # bus avail @ CRBT2
m.bd   = pyo.Var(T, B, domain=pyo.Binary)           # depart CEI
m.bd_r = pyo.Var(T, B, domain=pyo.Binary)           # depart CRBT2
m.Dep   = pyo.Var(P,   domain=pyo.NonNegativeReals) # depart time (slots)
m.Dep_r = pyo.Var(P_r, domain=pyo.NonNegativeReals)
# เวลารอ (หน่วย "ช่อง") — ใช้เป็น objective linear
m.s    = pyo.Var(P,   domain=pyo.NonNegativeReals)
m.s_r  = pyo.Var(P_r, domain=pyo.NonNegativeReals)

# ---------- Objective: minimize total waiting time (linear) ----------
m.obj = pyo.Objective(
    expr=sum(m.s[i] for i in P) + sum(m.s_r[i] for i in P_r),
    sense=pyo.minimize
)

# ---------- Constraints ----------

# F1, F4: assign exactly once
m.assign = pyo.ConstraintList()
for i in P:
    m.assign.add(sum(m.x[i,c,t] for c in B for t in T) == 1)
for i in P_r:
    m.assign.add(sum(m.x_r[i,c,t] for c in B for t in T) == 1)

# F2,F3,F5,F6: capacity linked to departures
m.cap = pyo.ConstraintList()
for t in T:
    for c in B:
        m.cap.add(sum(m.x[i,c,t]   for i in P)   <= c_max * m.bd[t,c])
        m.cap.add(sum(m.x_r[i,c,t] for i in P_r) <= c_max * m.bd_r[t,c])

# F7,F8: depart only if bus available
for t in T:
    for c in B:
        m.cap.add(m.bd[t,c]   <= m.ba[t,c])
        m.cap.add(m.bd_r[t,c] <= m.ba_r[t,c])

# F9–F12: flow equalities across time (index ด้วย k เพื่ออ้าง t_prev / t - tau)
m.flow = pyo.ConstraintList()
for k, t in enumerate(T_list):
    if k == 0:
        # F11: initial location
        for c in B:
            m.flow.add(m.ba[t,c] + m.ba_r[t,c] == 1)
        continue
    t_prev = T_list[k-1]
    for c in B:
        # CEI
        if k - tau_slots >= 0:
            t_arr_from_r = T_list[k - tau_slots]
            m.flow.add(m.ba[t,c]   == m.ba[t_prev,c]   - m.bd[t_prev,c]   + m.bd_r[t_arr_from_r,c])
        else:
            m.flow.add(m.ba[t,c]   == m.ba[t_prev,c]   - m.bd[t_prev,c])
        # CRBT2
        if k - tau_slots >= 0:
            t_arr_from_a = T_list[k - tau_slots]
            m.flow.add(m.ba_r[t,c] == m.ba_r[t_prev,c] - m.bd_r[t_prev,c] + m.bd[t_arr_from_a,c])
        else:
            m.flow.add(m.ba_r[t,c] == m.ba_r[t_prev,c] - m.bd_r[t_prev,c])

# M3: cannot be available at both stations simultaneously
for t in T:
    for c in B:
        m.flow.add(m.ba[t,c] + m.ba_r[t,c] <= 1)

# T1,T2: link Dep with chosen slot (Dep หน่วยเป็น "ช่อง")
for i in P:
    for c in B:
        for t in T:
            m.flow.add(m.Dep[i]   >= (t/STEP_MIN) - M*(1 - m.x[i,c,t]))
for i in P_r:
    for c in B:
        for t in T:
            m.flow.add(m.Dep_r[i] >= (t/STEP_MIN) - M*(1 - m.x_r[i,c,t]))

# s = waiting time = Dep - arr (bounded by w_max)
for i in P:
    m.flow.add(m.s[i]   >= m.Dep[i]   - arr_s[i])   # definition
    m.flow.add(m.s[i]   <= w_max)                   # F14
for i in P_r:
    m.flow.add(m.s_r[i] >= m.Dep_r[i] - arr_r_s[i])
    m.flow.add(m.s_r[i] <= w_max)                   # F15

# F10, F13: forbid boarding too late (ท้ายฮอไรซอนช่วง τ)
last_idx = list(range(max(0, N - tau_slots), N)) if tau_slots > 0 else []
for idx in last_idx:
    t_forbid = T_list[idx]
    for c in B:
        for i in P:
            m.flow.add(m.x[i,c,t_forbid] == 0)
        for i in P_r:
            m.flow.add(m.x_r[i,c,t_forbid] == 0)

# ---------- Solve with HiGHS ----------
solver = pyo.SolverFactory("highs")
# ตัวเลือกเสริม:
solver.options["threads"] = 20
res = solver.solve(m, tee=True)

print("Status:", res.solver.status, res.solver.termination_condition)
print("Objective (total wait slots):", pyo.value(m.obj))
for i in P:
    # หา t ที่เลือก
    chosen = [(c,t) for c in B for t in T if pyo.value(m.x[i,c,t]) > 0.5]
    if chosen:
        c,t = chosen[0]
        print(f"i={i} -> t={t:3d}  wait={pyo.value(m.s[i]):.1f} slots")
