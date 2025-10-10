import os
import time
import subprocess
from typing import List, Dict
from itertools import product
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm

PROJECT_PATH = "/home/hpcnc/bus_opt"
JULIA_BIN = os.path.expanduser("~/tool/julia/julia")

params = {
    "lambda_per_hour_cei": [75, 115, 150],
    "lambda_per_hour_t2": [40, 75, 115],
    "buses": [4, 5, 6, 7],
    "w_max" : [60, 75, 90]
}

# params = {
#     "lambda_per_hour_cei": [75],
#     "lambda_per_hour_t2": [40],
#     "buses": [4],
#     "w_max" : [60]
# }

def generate_permutation(params: dict) -> List[Dict[str, int]]:
    keys = list(params.keys())
    all_combos = product(*(params[k] for k in keys))
    return [dict(zip(keys, combo)) for combo in all_combos]

def ensure_dirs():
    os.makedirs(os.path.join(PROJECT_PATH, "case"), exist_ok=True)
    os.makedirs(os.path.join(PROJECT_PATH, "output"), exist_ok=True)
    os.makedirs(os.path.join(PROJECT_PATH, "log"), exist_ok=True)

def execute_case(p: dict):
    """Run one case (blocking). Raises on failure."""
    cei = p["lambda_per_hour_cei"]
    t2 = p["lambda_per_hour_t2"]
    w_max = p["w_max"]
    bus = p["buses"]

    tc_rel = f"case/tc_{cei}_{t2}_{w_max}_{bus}.json"
    out_rel = f"output/tc_{cei}_{t2}_{w_max}_{bus}.json"
    log_rel = f"log/tc_{cei}_{t2}_{w_max}_{bus}.txt"

    tc_abs  = os.path.join(PROJECT_PATH, tc_rel)
    out_abs = os.path.join(PROJECT_PATH, out_rel)
    log_abs = os.path.join(PROJECT_PATH, log_rel)

    if not os.path.exists(tc_abs):
        raise FileNotFoundError(f"Config not found: {tc_abs}")

    with open(log_abs, "w") as lf:
        subprocess.run(
            [JULIA_BIN, os.path.join(PROJECT_PATH, "bus2_opt.jl"), tc_abs, out_abs],
            stdout=lf,
            stderr=subprocess.STDOUT,
            check=True
        )
    return out_abs

def chunked(seq, n):
    """Yield lists of size n from seq."""
    for i in range(0, len(seq), n):
        yield seq[i:i+n]

def main():
    ensure_dirs()

    all_cases = generate_permutation(params=params)
    total = len(all_cases)

    max_workers = 4
    batch_size  = 4
    failures = []

    with tqdm(total=total) as pbar:
        for batch in chunked(all_cases, batch_size):
            with ThreadPoolExecutor(max_workers=max_workers) as ex:
                future_to_case = {ex.submit(execute_case, case): case for case in batch}
                for fut in as_completed(future_to_case):
                    case = future_to_case[fut]
                    try:
                        fut.result()
                    except Exception as e:
                        failures.append((case, str(e)))
                    finally:
                        pbar.update(1)

    if failures:
        print("\nSome cases failed:")
        for case, err in failures:
            print(f"  {case} -> {err}")
    else:
        print("\nAll cases completed successfully.")

if __name__ == "__main__":
    main()
