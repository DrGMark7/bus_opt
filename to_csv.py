import os
import json
import csv
from collections import defaultdict

folder_path = "output"
output_path = "output_csv"

header = ["time", "avg_wait_time", "max_wait_time"]

for filename in os.listdir(folder_path):
    if filename.endswith(".json"):
        file_path_tmp = os.path.join(folder_path, filename)
        output_path_tmp = os.path.join(output_path, filename[:-4]+"csv")

    with open(output_path_tmp, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

        with open(file_path_tmp, "r", encoding="utf-8") as f:
            try:
                print(f"====================== {file_path_tmp} ======================")
                data = json.load(f)
                interval_dict = defaultdict(list)
                status = data["status"]
                if status == "INFEASIBLE":
                    print("This case is INFEASIBLE")
                    continue

                assignments = data['assignments']
                for assign in assignments:
                    t = assign["t"]
                    interval_dict[t//60].append(assign["wait"])

                for t, wait_times in interval_dict.items():
                    max_wait_time = max(wait_times) if wait_times else 0
                    avg_wait_time = sum(wait_times)/len(wait_times) if wait_times else 0

                    print(f"At time interval {t:2} have max_wait_time: {max_wait_time:.2f}, avg_wait_time: {avg_wait_time:.2f}")
                
                    writer.writerow([t, avg_wait_time, max_wait_time])
                print("\n")
            except json.JSONDecodeError as e:
                print("Error decoding JSON:", e)

    with open(output_path_tmp, newline='') as csvfile:
        rows = list(csv.reader(csvfile, delimiter=',', quotechar='"'))
