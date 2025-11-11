import os
import time
import json
from typing import List, Dict
from itertools import product

# lambda = 40.1 for 500 passengers
# lambda = 59.4 for 750 passengers
# lambda = 82 for 1000 passengers
# lambda = 121.5 for 1500 passengers
# lambda = 165.4 for 2000 passengers


params = {
    "lambda_per_hour_cei": [121.5],
    "lambda_per_hour_t2": [82],
    "w_max" : [80],
    "buses" : [9, 10],
    "seed": [42]
}

# params = {
#     "lambda_per_hour_cei": [59.4, 40.1],
#     "lambda_per_hour_t2": [40.1],
#     "w_max" : [80],
#     "buses" : [4, 5, 6, 7],
#     "seed": [42]
# }

def get_base_case(seed: int, lmbda: int):
    try:
        filename = f"base_case/base_{seed}_{lmbda}.json"
        if not os.path.exists(filename):
            os.system(f"~/tool/julia/julia /home/hpcnc/bus_opt/make_base_case.jl --seed={seed} --lambda_per_hour={lmbda} --outpath={filename}")

    except Exception as e:
        raise e

    return filename

def gen_case(params: dict):
    cei = params["lambda_per_hour_cei"]
    t2 = params["lambda_per_hour_t2"]
    w_max = params["w_max"]
    bus = params["buses"]
    seed = params["seed"]

    mapp = {40.1: "500", 59.4:"750", 82: "1000", 121.5:"1500", 165.5: "2000"}

    base_case_cei = get_base_case(seed=seed, lmbda=cei)
    base_case_t2 = get_base_case(seed=seed, lmbda=t2)

    with open(base_case_cei, "r") as base_case_cei, open(base_case_t2, "r") as base_case_t2:
        case_cei = json.load(base_case_cei)
        case_t2 = json.load(base_case_t2)

        arrivals_cei = {"CEI": case_cei["arrivals"]["TERMINAL"]}
        arrivals_t2 = {"T2": case_t2["arrivals"]["TERMINAL"]}

        combined_arrivals = {**arrivals_cei, **arrivals_t2}
        
        combined_data = {
            "T": case_cei.get("T", case_t2.get("T")),
            "L": case_cei["L"],
            "tau": 40,
            "c_max": 20,
            "w_max": w_max,
            "B": [i+1 for i in range(bus)],
            "arrivals": combined_arrivals
        }
    out_file = f"final_case/tc_{mapp.get(cei, cei)}_{mapp.get(t2, t2)}_{w_max}_{bus}.json"
    with open(out_file, "w") as f_out:
        json.dump(combined_data, f_out, indent=4)

    print(f"Combined file saved to {out_file}")

def generate_permutation(params: dict) -> List[Dict[str, int]]:
    keys = list(params.keys())
    all_combos = product(*(params[k] for k in keys))
    return [dict(zip(keys, combo)) for combo in all_combos]

def main():
    possible_case = generate_permutation(params=params)
    for index, case in enumerate(possible_case):
        gen_case(case)

    
if __name__ == "__main__":
    main()
