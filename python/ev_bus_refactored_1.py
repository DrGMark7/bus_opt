
# ev_bus_refactored.py
# Refactored EV BUS scheduling model using NVIDIA cuOpt (linear_programming API).
# - Clean separation of config, data, model building, solving, and reporting
# - Implements constraints F1–F15, T1–T4, M1–M8 (as commonly defined in the paper)
# - Supports multiple buses (B >= 1). Set B=1 to emulate single-bus version.
#
# NOTE:
# - Objective uses linear waiting-time surrogate sum(s_i); switch to QP or PWL for squared waits.
# - Time unit is one slot; STEP_MIN defines minutes/slot when mapping to wall-clock minutes.
#
# Usage (demo at bottom):
#   python ev_bus_refactored.py
#
# Replace the synthetic arrivals with your real data (snap to grid) and re-run.
#
from dataclasses import dataclass
from typing import Dict, List, Tuple, Iterable, Optional
from cuopt.linear_programming.problem import Problem, INTEGER, CONTINUOUS, MINIMIZE
from cuopt.linear_programming.solver_settings import SolverSettings

# ---------- Data & Config ----------

@dataclass
class EVBusConfig:
    STEP_MIN: int = 1                 # minutes per time slot
    HORIZON_MIN: int = 200            # planning horizon in minutes
    TAU_MIN: int = 25                 # travel time A<->B in minutes
    CAPACITY: int = 50                # seats per bus per departure
    W_MAX_MIN: int = 60               # max allowed waiting in minutes
    BUSES: int = 3                    # number of buses
    INITIAL_AT_A: Optional[int] = None # how many buses start at A (if None, default: all at A)

    @property
    def T_slots(self) -> List[int]:
        return list(range(0, self.HORIZON_MIN + 1, self.STEP_MIN))

    @property
    def TAU_slots(self) -> int:
        return self.TAU_MIN // self.STEP_MIN

    @property
    def W_MAX_slots(self) -> int:
        return self.W_MAX_MIN // self.STEP_MIN

@dataclass
class Instance:
    # Passengers going A->B (CEI -> Terminal 2) and B->A (Terminal 2 -> CEI)
    P_A: List[int]
    P_B: List[int]
    # Arrivals on discrete grid (slot indices). Must satisfy arr[i] in T index-range
    arr_A: Dict[int, int]
    arr_B: Dict[int, int]

# ---------- Helpers ----------

def allowed_times(arrival_slot: int, cutoff: int) -> List[int]:
    '''Allowed boarding times: [arrival_slot, cutoff-1].'''
    return list(range(arrival_slot, cutoff))

# ---------- Model ----------

class EVBusModel:
    def __init__(self, cfg: EVBusConfig, data: Instance):
        self.cfg = cfg
        self.data = data
        self.T = cfg.T_slots
        self.NT = len(self.T)
        self.tau = cfg.TAU_slots
        self.wmax = cfg.W_MAX_slots
        self.BN = cfg.BUSES
        self.cutoff = self.NT - self.tau  # last slot you can depart and still arrive before horizon

        # cuOpt model container
        self.prob = Problem("EV_BUS")
        self.settings = SolverSettings()

        # Variables
        self._build_variables()
        # Constraints
        self._build_constraints()
        # Objective
        self._build_objective()

    # ---------- Variables ----------
    def _bin(self, name: str):
        return self.prob.addVariable(lb=0, ub=1, vtype=INTEGER, name=name)
    def _cont(self, lb: float, name: str, ub: float = 10_000.0):
        return self.prob.addVariable(lb=lb, ub=ub, vtype=CONTINUOUS, name=name)

    def _build_variables(self):
        P_A, P_B = self.data.P_A, self.data.P_B

        # Bus availability and departure decisions per bus, per time, per terminal
        self.ba   = {(b,t): self._bin(f"ba[{b},{t}]")   for b in range(self.BN) for t in range(self.NT)}  # A-side availability
        self.ba_r = {(b,t): self._bin(f"baB[{b},{t}]")  for b in range(self.BN) for t in range(self.NT)}  # B-side availability
        self.bd   = {(b,t): self._bin(f"bd[{b},{t}]")   for b in range(self.BN) for t in range(self.NT)}  # depart A at t
        self.bd_r = {(b,t): self._bin(f"bdB[{b},{t}]")  for b in range(self.BN) for t in range(self.NT)}  # depart B at t

        # Passenger assignment: picks a bus b and time t
        self.x_A = {}
        self.x_B = {}
        for i in P_A:
            for b in range(self.BN):
                for t in allowed_times(self.data.arr_A[i], self.cutoff):
                    self.x_A[(i,b,t)] = self._bin(f"xA[{i},{b},{t}]")
        for j in P_B:
            for b in range(self.BN):
                for t in allowed_times(self.data.arr_B[j], self.cutoff):
                    self.x_B[(j,b,t)] = self._bin(f"xB[{j},{b},{t}]")

        # Per-passenger departure times & waiting (slot units)
        self.Dep_A = {i: self._cont(0.0, f"DepA[{i}]") for i in P_A}
        self.Dep_B = {j: self._cont(0.0, f"DepB[{j}]") for j in P_B}
        self.s_A   = {i: self._cont(0.0, f"sA[{i}]")   for i in P_A}
        self.s_B   = {j: self._cont(0.0, f"sB[{j}]")   for j in P_B}

    # ---------- Constraints ----------
    def _build_constraints(self):
        cfg = self.cfg
        P_A, P_B = self.data.P_A, self.data.P_B
        NT, BN, tau, wmax = self.NT, self.BN, self.tau, self.wmax
        CAP = cfg.CAPACITY
        cutoff = self.cutoff

        # F1: Each A-passenger boards exactly once (some bus, some time)
        for i in P_A:
            self.prob.addConstraint(
                sum(self.x_A[(i,b,t)] for b in range(BN) for t in allowed_times(self.data.arr_A[i], cutoff)) == 1,
                name=f"F1_assign_A[{i}]"
            )

        # F4: Each B-passenger boards exactly once
        for j in P_B:
            self.prob.addConstraint(
                sum(self.x_B[(j,b,t)] for b in range(BN) for t in allowed_times(self.data.arr_B[j], cutoff)) == 1,
                name=f"F4_assign_B[{j}]"
            )

        # F2 & F3: Linking & Capacity @ A (per bus, per time)
        for b in range(BN):
            for t in range(NT):
                terms = [self.x_A[(i,b,t)] for i in P_A if (i,b,t) in self.x_A]
                if terms:
                    # Link: any boarding implies a departure
                    self.prob.addConstraint(sum(terms) <= CAP * self.bd[(b,t)], name=f"F2_link_A[{b},{t}]")
                    # Capacity
                    self.prob.addConstraint(sum(terms) <= CAP, name=f"F3_cap_A[{b},{t}]")

        # F5 & F6: Linking & Capacity @ B
        for b in range(BN):
            for t in range(NT):
                terms = [self.x_B[(j,b,t)] for j in P_B if (j,b,t) in self.x_B]
                if terms:
                    self.prob.addConstraint(sum(terms) <= CAP * self.bd_r[(b,t)], name=f"F5_link_B[{b},{t}]")
                    self.prob.addConstraint(sum(terms) <= CAP, name=f"F6_cap_B[{b},{t}]")

        # F7, F8: Depart only if bus is at that terminal
        for b in range(BN):
            for t in range(NT):
                self.prob.addConstraint(self.bd[(b,t)]   <= self.ba[(b,t)],   name=f"F7_depart_if_avail_A[{b},{t}]")
                self.prob.addConstraint(self.bd_r[(b,t)] <= self.ba_r[(b,t)], name=f"F8_depart_if_avail_B[{b},{t}]")

        # M2: At most one departure (A or B) per bus & time
        for b in range(BN):
            for t in range(NT):
                self.prob.addConstraint(self.bd[(b,t)] + self.bd_r[(b,t)] <= 1, name=f"M2_one_depart_per_bus[{b},{t}]")

        # M3: Bus can be at most one terminal at a time
        for b in range(BN):
            for t in range(NT):
                self.prob.addConstraint(self.ba[(b,t)] + self.ba_r[(b,t)] <= 1, name=f"M3_single_location[{b},{t}]")

        # F9–F14: Availability flow with travel-time delay
        #   ba[b,t+1] = ba[b,t] - bd[b,t] + bd_r[b,t-τ]   (if t-τ >= 0 else +0)
        #   ba_r[b,t+1] = ba_r[b,t] - bd_r[b,t] + bd[b,t-τ]
        for b in range(BN):
            for t in range(NT-1):
                inbound_from_B = self.bd_r[(b, t - tau)] if t - tau >= 0 else 0
                inbound_from_A = self.bd[(b, t - tau)]   if t - tau >= 0 else 0
                self.prob.addConstraint(
                    self.ba[(b,t+1)] == self.ba[(b,t)] - self.bd[(b,t)] + inbound_from_B,
                    name=f"Flow_A[{b},{t}]"
                )
                self.prob.addConstraint(
                    self.ba_r[(b,t+1)] == self.ba_r[(b,t)] - self.bd_r[(b,t)] + inbound_from_A,
                    name=f"Flow_B[{b},{t}]"
                )

        # F10 & F13: Forbid boarding too close to horizon
        for b in range(BN):
            for t in range(cutoff, NT):
                # No boarding assignments in tail (only add if there are terms; otherwise skip)
                terms_A = [self.x_A[(i,b,t)] for i in P_A if (i,b,t) in self.x_A]
                if terms_A:
                    self.prob.addConstraint(sum(terms_A) == 0, name=f"F10_no_board_A_late[{b},{t}]")
                terms_B = [self.x_B[(j,b,t)] for j in P_B if (j,b,t) in self.x_B]
                if terms_B:
                    self.prob.addConstraint(sum(terms_B) == 0, name=f"F13_no_board_B_late[{b},{t}]")
                # Optional: also prevent departures in tail if desired
                # self.prob.addConstraint(self.bd[(b,t)] == 0,   name=f"F10b_no_depart_A_late[{b},{t}]")
                # self.prob.addConstraint(self.bd_r[(b,t)] == 0, name=f"F13b_no_depart_B_late[{b},{t}]")

        # F15: Initial location(s)
        if cfg.INITIAL_AT_A is None:
            # default: all buses start at A
            self.prob.addConstraint(sum(self.ba[(b,0)] for b in range(BN)) == BN, name="F15_all_start_A")
            self.prob.addConstraint(sum(self.ba_r[(b,0)] for b in range(BN)) == 0,  name="F15_none_start_B")
        else:
            k = cfg.INITIAL_AT_A
            self.prob.addConstraint(sum(self.ba[(b,0)] for b in range(BN)) == k,         name="F15_k_start_A")
            self.prob.addConstraint(sum(self.ba_r[(b,0)] for b in range(BN)) == BN - k,  name="F15_rest_start_B")

        # T1/T2: Define departure time as convex combination of chosen times
        for i in P_A:
            self.prob.addConstraint(
                self.Dep_A[i] == sum(t * self.x_A[(i,b,t)] for b in range(BN) for t in allowed_times(self.data.arr_A[i], cutoff)),
                name=f"T1_dep_time_A[{i}]"
            )
        for j in P_B:
            self.prob.addConstraint(
                self.Dep_B[j] == sum(t * self.x_B[(j,b,t)] for b in range(BN) for t in allowed_times(self.data.arr_B[j], cutoff)),
                name=f"T2_dep_time_B[{j}]"
            )

        # T3/T4: Max waiting
        for i in P_A:
            self.prob.addConstraint(self.Dep_A[i] - self.data.arr_A[i] <= wmax, name=f"T3_wait_cap_A[{i}]")
        for j in P_B:
            self.prob.addConstraint(self.Dep_B[j] - self.data.arr_B[j] <= wmax, name=f"T4_wait_cap_B[{j}]")

        # Link waiting variables (non-negativity already by lb=0)
        for i in P_A:
            self.prob.addConstraint(self.s_A[i] >= self.Dep_A[i] - self.data.arr_A[i], name=f"Wait_def_A[{i}]")
        for j in P_B:
            self.prob.addConstraint(self.s_B[j] >= self.Dep_B[j] - self.data.arr_B[j], name=f"Wait_def_B[{j}]")

        # M7/M8 (typical): Disallow impossible combinations (already controlled by allowed_times)
        # You may add further logic here (e.g., min headway, maintenance windows).

    # ---------- Objective ----------
    def _build_objective(self):
        # Minimize total waiting time; swap to QP or PWL for squared waiting if needed.
        obj = sum(self.s_A.values()) + sum(self.s_B.values())
        self.prob.setObjective(obj, sense=MINIMIZE)

    # ---------- Solve & Report ----------
    def solve(self, start_at_A: Optional[List[int]] = None) -> Tuple:
        '''Optionally enforce specific buses to start at A (others at B) by fixing ba[0].'''
        if start_at_A is not None:
            # Override initial placement
            for b in range(self.BN):
                if b in start_at_A:
                    self.prob.addConstraint(self.ba[(b,0)] == 1, name=f"Init_at_A[{b}]")
                    self.prob.addConstraint(self.ba_r[(b,0)] == 0, name=f"Init_not_B[{b}]")
                else:
                    self.prob.addConstraint(self.ba[(b,0)] == 0, name=f"Init_not_A[{b}]")
                    self.prob.addConstraint(self.ba_r[(b,0)] == 1, name=f"Init_at_B[{b}]")

        res = self.prob.solve(self.settings)
        return res, self

    def chosen_departures(self) -> Dict[str, List[Tuple[int,int]]]:
        '''Return departures list per terminal: [(b,t), ...]'''
        dep_A = [(b,t) for (b,t),var in self.bd.items()   if var.getValue() > 0.5]
        dep_B = [(b,t) for (b,t),var in self.bd_r.items() if var.getValue() > 0.5]
        return {"A": dep_A, "B": dep_B}

    def chosen_assignments(self) -> Dict[str, List[Tuple[int,int,int]]]:
        '''Return passenger assignments as (id, bus, t).'''
        sel_A = [(i,b,t) for (i,b,t),var in self.x_A.items() if var.getValue() > 0.5]
        sel_B = [(j,b,t) for (j,b,t),var in self.x_B.items() if var.getValue() > 0.5]
        return {"A": sel_A, "B": sel_B}

    def print_summary(self):
        print("Status:", self.prob.Status.name)
        print("Objective:", self.prob.ObjValue)
        dep = self.chosen_departures()
        print("Departures A:", [(b, t*self.cfg.STEP_MIN) for (b,t) in dep["A"]])
        print("Departures B:", [(b, t*self.cfg.STEP_MIN) for (b,t) in dep["B"]])

        # Example: print first few assignments
        sel = self.chosen_assignments()
        for (i,b,t) in sel["A"][:5]:
            wait = self.Dep_A[i].getValue() - self.data.arr_A[i]
            print(f"A-passenger {i} -> bus {b} at t={t*self.cfg.STEP_MIN} min, wait={wait*self.cfg.STEP_MIN} min")
        for (j,b,t) in sel["B"][:5]:
            wait = self.Dep_B[j].getValue() - self.data.arr_B[j]
            print(f"B-passenger {j} -> bus {b} at t={t*self.cfg.STEP_MIN} min, wait={wait*self.cfg.STEP_MIN} min")

# ---------- Demo (synthetic) ----------

def _demo():
    cfg = EVBusConfig(
        STEP_MIN=1, HORIZON_MIN=200, TAU_MIN=25, CAPACITY=50, W_MAX_MIN=60,
        BUSES=2, INITIAL_AT_A=2  # two buses both start at A
    )
    T = cfg.T_slots
    tau = cfg.TAU_slots
    cutoff = len(T) - tau

    # Synthetic passengers
    P_A = list(range(1, 10))  # CEI -> B
    P_B = list(range(1, 2))  # B -> CEI
    arr_A = {i: (2*i) % (cutoff-1) for i in P_A}
    arr_B = {j: (3*j) % (cutoff-1) for j in P_B}

    data = Instance(P_A=P_A, P_B=P_B, arr_A=arr_A, arr_B=arr_B)
    model = EVBusModel(cfg, data)

    # Optionally pin starting locations (redundant because INITIAL_AT_A=2):
    # res, _ = model.solve(start_at_A=[0,1])
    _, _ = model.solve()
    try:
        print("Status:", model.prob.Status)
        print("Objective:", model.prob.ObjValue)
        model.print_summary()
    except Exception as e:
        print("Error occurred:", e)

if __name__ == "__main__":
    _demo()
