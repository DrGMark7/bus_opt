import os
import json
import csv
from collections import defaultdict

folder_path = "output"
output_path = "summary_csv"
os.makedirs(output_path, exist_ok=True)

output_file = os.path.join(output_path, "summary.csv")

header = [
    "filename",
    "runtime_seconds",
    "avg_wait_time_seconds",
    "max_wait_time_seconds",
    "avg_max_wait_time_seconds",
    "total_bus_trips",
    "deadhead_count_before_last",
    "avg_bus_idle_time"
]

with open(output_file, "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(header)

    for filename in os.listdir(folder_path):
        if not filename.endswith(".json"):
            continue

        file_path = os.path.join(folder_path, filename)

        try:
            print(f"====================== {file_path} ======================")
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            if data.get("status") == "INFEASIBLE":
                print("This case is INFEASIBLE")
                continue

            runtime = data.get("runtime", 0)

            # --- Wait time calculations ---
            assignments = data.get("assignments", [])
            interval_dict = defaultdict(list)

            for assign in assignments:
                t = assign["t"]
                wait = assign.get("wait", 0)
                interval_dict[t // 60].append(wait)

            avg_wait_time_list = []
            max_wait_time_list = []

            for wait_times in interval_dict.values():
                if not wait_times:
                    continue
                max_wait_time = max(wait_times)
                avg_wait_time = sum(wait_times) / len(wait_times)
                avg_wait_time_list.append(avg_wait_time)
                max_wait_time_list.append(max_wait_time)

            avg_wait_time = sum(avg_wait_time_list) / len(avg_wait_time_list) if avg_wait_time_list else 0
            avg_max_wait_time = sum(max_wait_time_list) / len(max_wait_time_list) if max_wait_time_list else 0
            max_wait_time = max(max_wait_time_list) if max_wait_time_list else 0

            # --- Deadhead calculation (only before last passenger departure) ---
            departures = data.get("departures", [])
            assign_keys = {(a["t"], a["terminal"], a["bus"]) for a in assignments}

            # Find last passenger time per bus
            last_passenger_time = defaultdict(lambda: -1)
            for a in assignments:
                last_passenger_time[a["bus"]] = max(last_passenger_time[a["bus"]], a["t"])

            deadhead_count_before_last = 0
            total_bus_trips = 0

            # Store departures by bus
            bus_departure_times = defaultdict(list)

            for d in departures:
                t, terminal, bus = d["t"], d["terminal"], d["bus"]
                if t < last_passenger_time[bus]:  # only count trips before last passenger
                    total_bus_trips += 1
                    bus_departure_times[bus].append(t)
                    if (t, terminal, bus) not in assign_keys:  # unassigned → deadhead
                        deadhead_count_before_last += 1

            # --- Calculate avg bus idle time before last passenger depart ---
            bus_idle_times = []
            for bus, times in bus_departure_times.items():
                times.sort()
                if len(times) > 1:
                    # print(times[1:])
                    # print("")
                    # print(times[:-1])
                    gaps = [t2 - t1 for t1, t2 in zip(times[:-1], times[1:])]
                    bus_idle_times.append(sum(gaps) / len(gaps))
            avg_bus_idle_time = (sum(bus_idle_times) / len(bus_idle_times))-40 if bus_idle_times else 0

            # --- Write CSV row ---
            writer.writerow([
                filename,
                round(runtime, 2),
                round(avg_wait_time, 2),
                round(max_wait_time, 2),
                round(avg_max_wait_time, 2),
                total_bus_trips,
                deadhead_count_before_last,
                round(avg_bus_idle_time,2)
            ])

            print(f"{filename}: avg_wait={avg_wait_time:.2f}, max_wait={max_wait_time:.2f}, total_bus_trips={total_bus_trips:.2f} , deadhead(before_last)={deadhead_count_before_last}")

        except json.JSONDecodeError as e:
            print(f"Error decoding {filename}: {e}")
        except Exception as e:
            print(f"Error processing {filename}: {e}")
