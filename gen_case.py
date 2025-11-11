import os
import time
from typing import List, Dict
from itertools import product

# lambda = 41.67 for 500 passenger
# lambda = 83.33 for 500 passenger
params = {
    "lambda_per_hour_cei": [2],
    "lambda_per_hour_t2": [1],
    "buses": [7],
    "w_max" : [80]
}

def generate_permutation(params: dict) -> List[Dict[str, int]]:
    keys = list(params.keys())
    all_combos = product(*(params[k] for k in keys))
    return [dict(zip(keys, combo)) for combo in all_combos]

def execute_case(params: dict):
    cei = params["lambda_per_hour_cei"]
    t2 = params["lambda_per_hour_t2"]
    w_max = params["w_max"]
    bus = params["buses"]

    outpath = f"case/tc_{cei}_{t2}_{w_max}_{bus}.json"
    os.system(f"~/tool/julia/julia /home/hpcnc/bus_opt/make_poisson.jl --lambda_per_hour_cei={cei} --lambda_per_hour_t2={t2} --buses={bus} --w_max={w_max} --outpath={outpath}")
    
possible_case = generate_permutation(params=params)
for index, case in enumerate(possible_case):
    execute_case(case)
    
