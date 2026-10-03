import csv
import json
from pathlib import Path
from .evaluation import summarize


def save_results(results, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "results.json").write_text(json.dumps(results, indent=2, allow_nan=False))
    (output / "summary.json").write_text(json.dumps(summarize(results), indent=2, allow_nan=False))
    from .aggregation import table_rows
    table = table_rows(results)
    (output / "paper_table.json").write_text(json.dumps({"aggregation": "trial means per object, object means per split, equal S0/S1 Avg",
        "units": "rates in percent; times in seconds", "scope": "Evaluated replay measurements",
        "execution_clock": "simulated trajectory duration; simulator wall time reported separately", "rows": table}, indent=2, allow_nan=False))
    with (output / "paper_table.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["variant", "split", "objects", "trials", "SR", "Tplan", "Texec", "Ttot", "Fplan", "Freach", "Fsafe", "Fstab", "Fafford"])
        writer.writeheader()
        writer.writerows(table)
    with (output / "results.csv").open("w", newline="") as stream:
        fields = ["trial_id", "object_id", "variant", "split", "success", "first_failure",
                  "stability", "plan", "reach", "affordance", "safe", "reference_outcome",
                  "right_pad_distance_m", "left_pad_distance_m", "planning_time_s", "execution_time_s", "total_time_s", "execution_wall_time_s"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            row = {k: result.get(k, result["metrics"].get(k)) for k in fields}
            row['reference_outcome'] = (result.get('replay_reference') or {}).get('assigned_outcome')
            distances = result.get('grasp_contact', {}).get('bilateral_distance_m')
            if distances:
                row['right_pad_distance_m'], row['left_pad_distance_m'] = distances
            writer.writerow(row)
    from .report import write_report
    write_report(results, output)
