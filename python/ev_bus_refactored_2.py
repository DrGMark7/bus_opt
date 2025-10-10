# ev_bus_refactored_1.py
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from cuopt.linear_programming.problem import Problem, INTEGER, CONTINUOUS, MINIMIZE
from cuopt.linear_programming.solver_settings import SolverSettings

@dataclass
class EVBusConfig:
    STEP_MIN: int = 1
    HORIZON_MIN: int = 10
    TAU_MIN: int = 25
    CAPACITY: int = 50
    W_MAX_MIN: int = 60
    BUSES: int = 3
    INITIAL_AT_A: Optional[int] = None

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
    P_A: List[int]
    P_B: List[int]
    arr_A: Dict[int, int]
    arr_B: Dict[int, int]

def allowed_times(arrival_slot: int, cutoff: int) -> List[int]:
    return list(range(arrival_slot, cutoff))

def pwl_square_breakpoints(wmax: int, step: int = 1) -> List[int]:
    return list(range(0, wmax + 1, step))

class EVBusModel:
    def __init__(self, cfg: EVBusConfig, data: Instance):
        self.cfg = cfg
        self.data = data
        self.T = cfg.T_slots
        self.NT = len(self.T)
        self.tau = cfg.TAU_slots
        self.wmax = cfg.W_MAX_slots
        self.BN = cfg.BUSES
        self.cutoff = self.NT - self.tau

        self.prob = Problem("EV_BUS")
        self.settings = SolverSettings()

        self._build_variables()
        self._build_constraints()
        self._build_objective()

    def _bin(self, name: str):
        return self.prob.addVariable(lb=0, ub=1, vtype=INTEGER, name=name)

    def _cont(self, lb: float, name: str, ub: float = 10_000.0):
        return self.prob.addVariable(lb=lb, ub=ub, vtype=CONTINUOUS, name=name)

    def _build_variables(self):
        P_A, P_B = self.data.P_A, self.data.P_B

        self.ba   = {(b,t): self._bin(f"ba[{b},{t}]")   for b in range(self.BN) for t in range(self.NT)}
        self.ba_r = {(b,t): self._bin(f"baB[{b},{t}]")  for b in range(self.BN) for t in range(self.NT)}
        self.bd   = {(b,t): self._bin(f"bd[{b},{t}]")   for b in range(self.BN) for t in range(self.NT)}
        self.bd_r = {(b,t): self._bin(f"bdB[{b},{t}]")  for b in range(self.BN) for t in range(self.NT)}

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

        self.Dep_A = {i: self._cont(0.0, f"DepA[{i}]") for i in P_A}
        self.Dep_B = {j: self._cont(0.0, f"DepB[{j}]") for j in P_B}
        self.s_A   = {i: self._cont(0.0, f"sA[{i}]")   for i in P_A}
        self.s_B   = {j: self._cont(0.0, f"sB[{j}]")   for j in P_B}

        self.q_A = {i: self._cont(0.0, f"qA[{i}]") for i in P_A}
        self.q_B = {j: self._cont(0.0, f"qB[{j}]") for j in P_B}

        self._bp = pwl_square_breakpoints(self.wmax, step=5)
        self._K = len(self._bp)
        self.lam_A = {(i,k): self._cont(0.0, f"lamA[{i},{k}]", ub=1.0) for i in P_A for k in range(self._K)}
        self.lam_B = {(j,k): self._cont(0.0, f"lamB[{j},{k}]", ub=1.0) for j in P_B for k in range(self._K)}

    def _build_constraints(self):
        cfg = self.cfg
        P_A, P_B = self.data.P_A, self.data.P_B
        NT, BN, tau, wmax = self.NT, self.BN, self.tau, self.wmax
        CAP = cfg.CAPACITY
        cutoff = self.cutoff

        for i in P_A:
            self.prob.addConstraint(
                sum(self.x_A[(i,b,t)] for b in range(BN) for t in allowed_times(self.data.arr_A[i], cutoff)) == 1,
                name=f"F1_assign_A[{i}]"
            )

        for j in P_B:
            self.prob.addConstraint(
                sum(self.x_B[(j,b,t)] for b in range(BN) for t in allowed_times(self.data.arr_B[j], cutoff)) == 1,
                name=f"F4_assign_B[{j}]"
            )

        for b in range(BN):
            for t in range(NT):
                terms = [self.x_A[(i,b,t)] for i in P_A if (i,b,t) in self.x_A]
                if terms:
                    self.prob.addConstraint(sum(terms) <= CAP * self.bd[(b,t)], name=f"F2_link_A[{b},{t}]")
                    self.prob.addConstraint(sum(terms) <= CAP, name=f"F3_cap_A[{b},{t}]")

        for b in range(BN):
            for t in range(NT):
                terms = [self.x_B[(j,b,t)] for j in P_B if (j,b,t) in self.x_B]
                if terms:
                    self.prob.addConstraint(sum(terms) <= CAP * self.bd_r[(b,t)], name=f"F5_link_B[{b},{t}]")
                    self.prob.addConstraint(sum(terms) <= CAP, name=f"F6_cap_B[{b},{t}]")

        for b in range(BN):
            for t in range(NT):
                self.prob.addConstraint(self.bd[(b,t)]   <= self.ba[(b,t)],   name=f"F7_depart_if_avail_A[{b},{t}]")
                self.prob.addConstraint(self.bd_r[(b,t)] <= self.ba_r[(b,t)], name=f"F8_depart_if_avail_B[{b},{t}]")

        for b in range(BN):
            for t in range(NT):
                self.prob.addConstraint(self.bd[(b,t)] + self.bd_r[(b,t)] <= 1, name=f"M2_one_depart_per_bus[{b},{t}]")

        for b in range(BN):
            for t in range(NT):
                self.prob.addConstraint(self.ba[(b,t)] + self.ba_r[(b,t)] <= 1, name=f"M3_single_location[{b},{t}]")

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

        for b in range(BN):
            for t in range(cutoff, NT):
                terms_A = [self.x_A[(i,b,t)] for i in P_A if (i,b,t) in self.x_A]
                if terms_A:
                    self.prob.addConstraint(sum(terms_A) == 0, name=f"F10_no_board_A_late[{b},{t}]")
                terms_B = [self.x_B[(j,b,t)] for j in P_B if (j,b,t) in self.x_B]
                if terms_B:
                    self.prob.addConstraint(sum(terms_B) == 0, name=f"F13_no_board_B_late[{b},{t}]")

        if cfg.INITIAL_AT_A is None:
            self.prob.addConstraint(sum(self.ba[(b,0)] for b in range(BN)) == BN, name="F15_all_start_A")
            self.prob.addConstraint(sum(self.ba_r[(b,0)] for b in range(BN)) == 0,  name="F15_none_start_B")
        else:
            k = cfg.INITIAL_AT_A
            self.prob.addConstraint(sum(self.ba[(b,0)] for b in range(BN)) == k,    name="F15_k_start_A")
            self.prob.addConstraint(sum(self.ba_r[(b,0)] for b in range(BN)) == BN - k,  name="F15_rest_start_B")

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

        for i in P_A:
            self.prob.addConstraint(self.Dep_A[i] - self.data.arr_A[i] <= wmax, name=f"T3_wait_cap_A[{i}]")
        for j in P_B:
            self.prob.addConstraint(self.Dep_B[j] - self.data.arr_B[j] <= wmax, name=f"T4_wait_cap_B[{j}]")

        for i in P_A:
            self.prob.addConstraint(self.s_A[i] >= self.Dep_A[i] - self.data.arr_A[i], name=f"Wait_def_A[{i}]")
        for j in P_B:
            self.prob.addConstraint(self.s_B[j] >= self.Dep_B[j] - self.data.arr_B[j], name=f"Wait_def_B[{j}]")

        for i in P_A:
            self.prob.addConstraint(
                sum(self.lam_A[(i,k)] for k in range(self._K)) == 1.0,
                name=f"PWL_A_sumlam[{i}]"
            )
            self.prob.addConstraint(
                self.s_A[i] == sum(self._bp[k] * self.lam_A[(i,k)] for k in range(self._K)),
                name=f"PWL_A_s_def[{i}]"
            )
            self.prob.addConstraint(
                self.q_A[i] == sum((self._bp[k] ** 2) * self.lam_A[(i,k)] for k in range(self._K)),
                name=f"PWL_A_q_def[{i}]"
            )

        for j in P_B:
            self.prob.addConstraint(
                sum(self.lam_B[(j,k)] for k in range(self._K)) == 1.0,
                name=f"PWL_B_sumlam[{j}]"
            )
            self.prob.addConstraint(
                self.s_B[j] == sum(self._bp[k] * self.lam_B[(j,k)] for k in range(self._K)),
                name=f"PWL_B_s_def[{j}]"
            )
            self.prob.addConstraint(
                self.q_B[j] == sum((self._bp[k] ** 2) * self.lam_B[(j,k)] for k in range(self._K)),
                name=f"PWL_B_q_def[{j}]"
            )

    def _build_objective(self):
        obj = sum(self.q_A.values()) + sum(self.q_B.values())
        self.prob.setObjective(obj, sense=MINIMIZE)

    def solve(self, start_at_A: Optional[List[int]] = None) -> Tuple[object, "EVBusModel"]:
        if start_at_A is not None:
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
        dep_A = [(b,t) for (b,t),var in self.bd.items()   if var.getValue() > 0.5]
        dep_B = [(b,t) for (b,t),var in self.bd_r.items() if var.getValue() > 0.5]
        return {"A": dep_A, "B": dep_B}

    def chosen_assignments(self) -> Dict[str, List[Tuple[int,int,int]]]:
        sel_A = [(i,b,t) for (i,b,t),var in self.x_A.items() if var.getValue() > 0.5]
        sel_B = [(j,b,t) for (j,b,t),var in self.x_B.items() if var.getValue() > 0.5]
        return {"A": sel_A, "B": sel_B}

    def print_summary(self):
        print("Status:", self.prob.Status.name)
        print("Objective:", self.prob.ObjValue)
        dep = self.chosen_departures()
        print("Departures A:", [(b, t*self.cfg.STEP_MIN) for (b,t) in dep["A"]])
        print("Departures B:", [(b, t*self.cfg.STEP_MIN) for (b,t) in dep["B"]])
        sel = self.chosen_assignments()
        for (i,b,t) in sel["A"][:5]:
            wait = self.Dep_A[i].getValue() - self.data.arr_A[i]
            print(f"A-passenger {i} -> bus {b} at t={t*self.cfg.STEP_MIN} min, wait={wait*self.cfg.STEP_MIN} min")
        for (j,b,t) in sel["B"][:5]:
            wait = self.Dep_B[j].getValue() - self.data.arr_B[j]
            print(f"B-passenger {j} -> bus {b} at t={t*self.cfg.STEP_MIN} min, wait={wait*self.cfg.STEP_MIN} min")

def _demo():
    cfg = EVBusConfig(
        STEP_MIN=1, HORIZON_MIN=200, TAU_MIN=25, CAPACITY=50, W_MAX_MIN=60,
        BUSES=2, INITIAL_AT_A=2
    )
    T = cfg.T_slots
    tau = cfg.TAU_slots
    cutoff = len(T) - tau

    P_A = list(range(1, 21))
    P_B = list(range(1, 4))
    arr_A = {i: (2*i) % (cutoff-1) for i in P_A}
    arr_B = {j: (3*j) % (cutoff-1) for j in P_B}

    data = Instance(P_A=P_A, P_B=P_B, arr_A=arr_A, arr_B=arr_B)
    model = EVBusModel(cfg, data)
    _ , _ = model.solve()
    try:
        print("Status:", model.prob.Status)
        print("Objective:", model.prob.ObjValue)
        model.print_summary()
    except Exception as e:
        print("Error occurred:", e)

if __name__ == "__main__":
    _demo()