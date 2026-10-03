"""Table I aggregation: trials -> object means -> split means -> equal split Avg."""
from collections import defaultdict
import numpy as np

FAILURES = {"Fplan": "plan", "Freach": "reach", "Fsafe": "safe", "Fstab": "stability", "Fafford": "affordance"}
TIMES = {"Tplan": "planning_time_s", "Texec": "execution_time_s", "Ttot": "total_time_s"}


def table_rows(results):
    groups = defaultdict(lambda: defaultdict(list))
    for row in results:
        groups[(row["variant"], row["split"])][row["object_id"]].append(row)
    rows = []
    for (method, split), objects in sorted(groups.items()):
        means = []
        for trials in objects.values():
            value = {"SR": 100*sum(r["success"] for r in trials)/len(trials)}
            for name, failure in FAILURES.items():
                value[name] = None if split == "S0" and name == "Fafford" else 100*sum(r["first_failure"] == failure for r in trials)/len(trials)
            for name, source in TIMES.items():
                timings = [r.get(source) for r in trials]
                value[name] = float(np.mean(timings)) if all(t is not None for t in timings) else None
            means.append(value)
        row = {"variant": method, "split": split, "objects": len(objects), "trials": sum(map(len, objects.values()))}
        for key in ["SR", *TIMES, *FAILURES]:
            values = [x[key] for x in means]
            row[key] = float(np.mean(values)) if all(v is not None for v in values) else None
        rows.append(row)
    for method in sorted({r["variant"] for r in rows}):
        pair = [r for r in rows if r["variant"] == method and r["split"] in ("S0", "S1")]
        if len(pair) != 2:
            continue
        avg = {"variant": method, "split": "Avg", "objects": sum(r["objects"] for r in pair),
               "trials": sum(r["trials"] for r in pair)}
        for key in ["SR", *TIMES, *FAILURES]:
            values = [0. if key == "Fafford" and r["split"] == "S0" else r[key] for r in pair]
            avg[key] = float(np.mean(values)) if all(v is not None for v in values) else None
        rows.append(avg)
    return rows
