"""Table-conditioned replay records, kept separate from measured evaluation."""
from collections import Counter, defaultdict
from copy import deepcopy
import csv
import gzip
import html
from importlib.resources import files
import json
from pathlib import Path
import numpy as np
from .aggregation import table_rows
from .demos import load_demo, from_selection
from .geometry import transform
from .robot import HOME, GOAL

KIND = "reconstructed_replay"
FAILURES = {"Fplan": "plan", "Freach": "reach", "Fsafe": "safe", "Fstab": "stability", "Fafford": "affordance"}
METRICS = ["SR", "Tplan", "Texec", "Ttot", *FAILURES]


def reference():
    return json.loads(files(__package__).joinpath("assets", "paper_reference.json").read_text())


def write_records(path, records):
    # Stable compressed bytes, including gzip timestamp and filename.
    with Path(path).open("wb") as stream, gzip.GzipFile(filename="", fileobj=stream, mode="wb", mtime=0) as zipped:
        for record in records:
            zipped.write((json.dumps(record, separators=(",", ":"), allow_nan=False)+"\n").encode())


def read_records(path=None):
    content = Path(path).read_bytes() if path else files(__package__).joinpath("assets", "paper_replay.jsonl.gz").read_bytes()
    return [json.loads(line) for line in gzip.decompress(content).decode().splitlines()]


def allocate_time(mean_s, outcomes, excluded, rng):
    """Microsecond allocation with an exact published all-trial mean."""
    weights = rng.uniform(.65, 1.35, len(outcomes))
    weights[[i for i, o in enumerate(outcomes) if o in excluded]] = 0
    total = round(mean_s*1_000_000*len(outcomes))
    raw = weights/weights.sum()*total
    integer = np.floor(raw).astype(np.int64)
    order = np.argsort(-(raw-integer), kind="stable")
    integer[order[:total-int(integer.sum())]] += 1
    return integer


def trajectory(outcome, method_index, rng):
    """Compact, time-normalized UR5e joint keyframes for offline playback."""
    start, end = HOME.copy(), GOAL.copy()
    if outcome in ("stability", "plan"):
        return [start.tolist(), start.tolist()]
    if outcome == "reach":
        end = start + .2*(end-start)
    u = np.array([0., .2, .5, .8, 1.])
    q = start+u[:, None]*(end-start)
    bend = rng.uniform(-.025, .025, 6)
    bend[0] += [-.025, .025, -.01, .01][method_index]
    q += np.sin(np.pi*u)[:, None]*bend
    if outcome == "safe":
        # This shape depicts an approach deviation; actual contacts are queried
        # independently when a record is replayed with a concrete object/hand.
        unsafe = load_demo("hammer", "execution_deviation")["executed_joints"]
        q = np.asarray([unsafe[i] for i in np.linspace(0, len(unsafe)-1, 5).astype(int)])
    return q.round(9).tolist()


def generate(seed=0):
    ref = reference(); n = ref["replay"]["trials_per_method_split"]
    rng = np.random.default_rng(seed); records = []
    methods = list(ref["methods"])
    for row in ref["rows"]:
        if row["split"] == "Avg": continue
        objects = ref["splits"][row["split"]]
        if n % len(objects): raise ValueError("Replay count must divide equally over objects")
        counts = {"success": round(n*row["SR"]/100)}
        counts.update({failure: round(n*(row[key] or 0)/100) for key, failure in FAILURES.items()})
        if sum(counts.values()) != n: raise ValueError("Source rates do not sum to 100%")
        outcomes = [o for o, count in counts.items() for _ in range(count)]
        rng.shuffle(outcomes)
        plan_us = allocate_time(row["Tplan"], outcomes, {"stability"}, rng)
        exec_us = allocate_time(row["Texec"], outcomes, {"stability", "plan"}, rng)
        mi = methods.index(row["method"])
        for i, outcome in enumerate(outcomes):
            obj = objects[i % len(objects)]
            tp, te = int(plan_us[i])/1e6, int(exec_us[i])/1e6
            knots = trajectory(outcome, mi, rng)
            records.append({"schema_version": "handover.paper_replay_record.v1", "record_kind": KIND,
                "id": f"{row['split']}_m{mi+1}_{i:04d}", "method": row["method"], "split": row["split"],
                "object_id": obj, "pair_id": f"{row['split']}_{i:04d}", "hand": "left" if (i//len(objects)+i%len(objects))%2 else "right",
                "receiver_sequence_index": (i//2)%200,
                "source_table": "I", "setting_ref": f"{row['split']}/{row['method']}", "seed": seed,
                "outcome": outcome, "outcome_source": "paper-conditioned allocation",
                "stage_time_s": {"plan": tp, "execute": te, "total": round(tp+te, 6)},
                "trajectory": {"units": "rad", "interpolation": "linear", "time_s": np.linspace(0, te, len(knots)).round(6).tolist(),
                               "joints": knots, "starts_after_plan_s": tp},
                "events": [{"time_s": round(tp+te, 6), "kind": "reference_outcome", "value": outcome}]})
    return records


def aggregate(records):
    rows = []
    for r in records:
        t = r["stage_time_s"]
        rows.append({"variant": r["method"], "split": r["split"], "object_id": r["object_id"],
            "success": r["outcome"] == "success", "first_failure": None if r["outcome"] == "success" else r["outcome"],
            "planning_time_s": t["plan"], "execution_time_s": t["execute"], "total_time_s": t["total"]})
    return table_rows(rows)


def verify(records):
    ref = reference(); seen = set(); setting_counts = Counter()
    valid_methods, valid_outcomes = set(ref["methods"]), {"success", *FAILURES.values()}
    for r in records:
        if r["id"] in seen: raise ValueError("Duplicate replay ID")
        seen.add(r["id"])
        if r.get("record_kind") != KIND: raise ValueError("Missing reconstructed replay provenance")
        if r["method"] not in valid_methods or r["split"] not in ref["splits"]: raise ValueError("Unknown paper setting")
        if r["object_id"] not in ref["splits"][r["split"]]: raise ValueError("Object/split mismatch")
        if r["outcome"] not in valid_outcomes or (r["split"] == "S0" and r["outcome"] == "affordance"):
            raise ValueError("Outcome contradicts split protocol")
        q = np.asarray(r["trajectory"]["joints"]); times = np.asarray(r["trajectory"]["time_s"])
        t = r["stage_time_s"]
        if q.shape != (len(times), 6) or len(times) < 2 or not np.isfinite(q).all() or np.any(np.abs(q)>2*np.pi):
            raise ValueError("Invalid joint trajectory")
        if not np.isfinite(times).all() or np.any(np.diff(times)<0) or times[0]!=0 or abs(times[-1]-t["execute"])>1e-6:
            raise ValueError("Invalid trajectory timing")
        if any(not np.isfinite(v) or v < 0 for v in t.values()) or abs(t["total"]-t["plan"]-t["execute"])>1e-6:
            raise ValueError("Invalid stage timing")
        setting_counts[(r["method"], r["split"], r["object_id"])] += 1
    n = ref["replay"]["trials_per_method_split"]
    for split, objects in ref["splits"].items():
        for method in valid_methods:
            if any(setting_counts[(method, split, obj)] != n//len(objects) for obj in objects):
                raise ValueError("Missing or unbalanced object records")
    rows = aggregate(records); actual = {(r["variant"], r["split"]): r for r in rows}
    errors = []
    for target in ref["rows"]:
        result = actual[(target["method"], target["split"])]
        for metric in METRICS:
            a, b = result[metric], target[metric]
            if (a is None) != (b is None) or (a is not None and abs(a-b)>1e-8):
                errors.append({"method": target["method"], "split": target["split"], "metric": metric, "replay": a, "paper": b})
    if errors: raise ValueError(f"Replay differs from Table I: {errors}")
    return {"status": "passed", "record_kind": KIND, "records": len(records), "table_rows": len(rows),
            "checked_table_cells": len(ref["rows"])*len(METRICS), "max_error": 0., "original_experiments_executed": False}


def export(output, seed=0):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    records = generate(seed); audit = verify(records); rows = aggregate(records); ref = reference()
    write_records(output/"records.jsonl.gz", records)
    (output/"reference.json").write_text(json.dumps(ref, indent=2))
    (output/"verification.json").write_text(json.dumps(audit, indent=2))
    (output/"summary.json").write_text(json.dumps({"record_kind": KIND, "rows": rows}, indent=2))
    with (output/"summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    grouped = defaultdict(list)
    for r in records: grouped[(r["method"], r["split"], r["outcome"])].append(r["id"])
    examples = [{"method": k[0], "split": k[1], "outcome": k[2], "record": v[0], "count": len(v)} for k, v in grouped.items()]
    (output/"examples.json").write_text(json.dumps(examples, indent=2))
    write_report(output, rows, examples)
    return audit


def write_report(output, rows, examples):
    page = '''<!doctype html><html><meta charset="utf-8"><title>Paper replay</title><style>body{font:15px system-ui;max-width:1400px;margin:32px auto;padding:0 20px;background:#101b2a;color:#edf3ff}table{border-collapse:collapse;width:100%}td,th{padding:10px;text-align:right;border-bottom:1px solid #34445c}a{color:#83caff}code{font-size:13px}p{line-height:1.6}</style><h1>R2HandoverSim / Paper replay</h1><p>Reconstructed replay conditioned on Table I. Source statistics and playback records are separate from simulator measurements.</p><p>8,000 records · 16 object IDs · 4 baseline settings · S0 / S1 / Avg</p><p><a href="records.jsonl.gz">Records</a> · <a href="summary.csv">CSV</a> · <a href="reference.json">Settings / source</a> · <a href="verification.json">Verification</a></p><table><tr><th>Method</th><th>Split</th>'''
    page += ''.join(f'<th>{k}</th>' for k in METRICS)+'</tr>'
    for r in rows:
        page += '<tr><td>'+html.escape(r['variant'])+'</td><td>'+r['split']+'</td>'
        page += ''.join('<td>'+('N/A' if r[k] is None else f'{r[k]:.2f}')+'</td>' for k in METRICS)+'</tr>'
    page += '</table><h2>Replay examples</h2><table><tr><th>Setting</th><th>Outcome</th><th>Count</th><th>Record ID</th></tr>'
    for r in examples:
        page += f'<tr><td>{r["split"]} / {html.escape(r["method"])}</td><td>{r["outcome"]}</td><td>{r["count"]}</td><td><code>{r["record"]}</code></td></tr>'
    page += '</table><p>Export any ID with <code>r2handoversim replay-trial --record ID --output trial.json</code>, then use <code>demo --trial trial.json</code> in Isaac Sim. Add <code>--dataset /path/to/dataset.json</code> to load local configured objects.</p></html>'
    (Path(output)/"report.html").write_text(page)


def materialize(record, dataset=None, samples=90):
    """Convert one compact record to an ordinary, independently evaluated trial."""
    outcome = record["outcome"]
    if dataset:
        root = Path(dataset).resolve().parent
        manifest = json.loads(Path(dataset).read_text())
        if manifest.get("schema_version") != "handover.dataset.v1": raise ValueError("Expected dataset manifest")
        source = next((r for r in manifest["objects"] if r["object_id"] == record["object_id"]), None)
        if source is None: raise ValueError(f"Configured object missing: {record['object_id']}")
        from .workflows import read_relative
        trial = from_selection(read_relative(root, source["scene"]), read_relative(root, source["selection"]))
        geometry_source = "local configured object and generated scene annotations"
    else:
        name = "bottle" if record["split"] == "S0" else "hammer"
        trial = load_demo(name)
        geometry_source = f"procedural {trial['object_id']} stand-in for {record['object_id']}"
    q = np.asarray(record["trajectory"]["joints"])
    executed = np.column_stack([np.interp(np.linspace(0, 1, samples), np.linspace(0, 1, len(q)), q[:, j]) for j in range(6)])
    trial.update(id=record["id"], object_id=record["object_id"], variant=record["method"], split=record["split"],
                 executed_joints=executed.tolist())
    trial["dt_s"] = record["stage_time_s"]["execute"]/(samples-1) if record["stage_time_s"]["execute"] else 1/60
    if outcome == "plan":
        trial["planned_joints"] = [HOME.tolist(), HOME.tolist()]
        target = transform(trial["target_T_world_gripper"]); target[0, 3] += 2
        trial["target_T_world_gripper"] = target.tolist()
    if outcome == "stability":
        # Closing along the longest object axis demonstrates the width predicate.
        from .geometry import corners
        xyz = np.concatenate([corners(b) for b in trial["object_boxes"]])
        axis = int(np.argmax(np.ptp(xyz, axis=0))); y = np.eye(3)[axis]; z = np.eye(3)[(axis+1)%3]
        grasp = transform(trial["T_object_gripper"]); grasp[:3, :3] = np.column_stack([np.cross(y, z), y, z])
        trial["T_object_gripper"] = grasp.tolist()
    if outcome == "affordance":
        # Reserve the region intersecting the selected fingers for this scenario.
        from .geometry import gripper_boxes, moved
        trial["usage_boxes"] = [moved(b, trial["T_object_gripper"]) for b in gripper_boxes(.04) if b["label"] == "finger"]
    trial["replay_reference"] = {"record_kind": KIND, "record_id": record["id"], "source_table": "I",
        "assigned_outcome": outcome, "reference_stage_time_s": record["stage_time_s"],
        "assigned_hand": record["hand"], "receiver_template_index": record["receiver_sequence_index"],
        "receiver_geometry": "supplied scene or demo; template index is replay metadata",
        "geometry_source": geometry_source, "setting": reference()["methods"][record["method"]],
        "trajectory_source": "reconstructed keyframes; measured simulator flags are evaluated independently"}
    trial["provenance"] = "Reconstructed paper replay"
    return trial
