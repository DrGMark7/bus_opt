import os
import time
from typing import List, Dict
from itertools import product
from tqdm import tqdm

PROJECT_PATH = "/home/hpcnc/bus_opt"

# params = {
#     "passengers_cei": [1000, 1500, 2000],
#     "passengers_t2": [500, 1000, 1500],
#     "buses": [6, 7],
#     "w_max" : [80]
# }

params = {
    "passengers_cei": [1],
    "passengers_t2": [0.5],
    "buses": [4],
    "w_max" : [20]
}

def generate_permutation(params: dict) -> List[Dict[str, int]]:
    keys = list(params.keys())
    all_combos = product(*(params[k] for k in keys))
    return [dict(zip(keys, combo)) for combo in all_combos]

def execute_case(params: dict):
    cei = params["passengers_cei"]
    t2 = params["passengers_t2"]
    w_max = params["w_max"]
    bus = params["buses"]

    tc = f"final_case/tc_{cei}_{t2}_{w_max}_{bus}.json"
    result = f"output/tc_{cei}_{t2}_{w_max}_{bus}.json"
    log = f"{PROJECT_PATH}/log/tc_{cei}_{t2}_{w_max}_{bus}.txt"
    
    os.system(f"~/tool/julia/julia {PROJECT_PATH}/bus2_opt.jl {tc} {result} > {log} 2>&1")


s = time.perf_counter()

time_list = {}
possible_case = generate_permutation(params=params)

for index, case in enumerate(tqdm(possible_case)):
    
    cei = case["passengers_cei"]
    t2 = case["passengers_t2"]
    w_max = case["w_max"]
    bus = case["buses"]

    start = time.perf_counter()
    execute_case(case)
    end = time.perf_counter()

    time_list[f"{cei}_{t2}_{w_max}_{bus}"] = end - start

    # if (index + 1) % 5 == 0:
    #     time.sleep(3.5 * 60)

print(f"Total time to Execute {time.perf_counter() - s:.3f} sec")
print(f"Average Case time {sum(time_list.values())/len(time_list.values()):.3f} sec")
print("========================================")

for case, t in time_list.items():
    print(f"Case {case} use time {t:.3f} sec")
