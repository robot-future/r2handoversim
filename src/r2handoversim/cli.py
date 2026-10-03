import argparse
import csv
import json
from pathlib import Path
from .demos import NAMES, VARIANTS, from_selection, load_demo
from .evaluation import evaluate, summarize, validate_trial
from .results import save_results


def main(argv=None):
    parser = argparse.ArgumentParser(description="R2HandoverSim: Isaac Sim replay demos and offline evaluation")
    from . import __version__
    parser.add_argument('--version', action='version', version=f'r2handoversim {__version__}')
    sub = parser.add_subparsers(dest="command", required=True)
    review = sub.add_parser('review-video',help='Caption verified Isaac recordings with receiver IDs and measured results')
    review.add_argument('--input',type=Path,required=True)
    review.add_argument('--output',type=Path,required=True)
    candidates = sub.add_parser('prepare-candidates', help='Fit all method candidates to original USD pads before selection (Isaac Sim Python)')
    candidates.add_argument('--scene',type=Path,required=True)
    candidates.add_argument('--asset-config',type=Path,required=True)
    candidates.add_argument('--output',type=Path,required=True)
    receivers = sub.add_parser('sample-receivers',help='Assign seeded, fixed-world receiving hand mesh poses to local-asset trials')
    receivers.add_argument('--trials',type=Path,required=True)
    receivers.add_argument('--receiver-config',type=Path,required=True)
    receivers.add_argument('--samples',type=int,default=4)
    receivers.add_argument('--seed',type=int,default=0)
    receivers.add_argument('--output',type=Path,default=Path('outputs/receiver_trials'))
    scenes = sub.add_parser('receiver-scenes',help='Generate fixed-world hand scenes for method selection before replay')
    scenes.add_argument('--scene',type=Path,required=True)
    scenes.add_argument('--receiver-config',type=Path,required=True)
    scenes.add_argument('--samples',type=int,default=4)
    scenes.add_argument('--seed',type=int,default=0)
    scenes.add_argument('--output',type=Path,default=Path('outputs/receiver_scenes'))
    doctor = sub.add_parser('doctor', help='Check this Python environment and optional assets without starting Kit')
    doctor.add_argument('--isaac', action='store_true', help='Require Isaac Sim to be installed')
    doctor.add_argument('--video', action='store_true', help='Require an H.264 ffmpeg encoder')
    doctor.add_argument('--asset-config', type=Path)
    doctor.add_argument('--output', type=Path, help='Save a machine-readable JSON check report')
    assets = sub.add_parser('from-assets', help='Generate standalone geometry demos from local OBJ meshes')
    assets.add_argument('--asset-config', type=Path, required=True)
    assets.add_argument('--objects', nargs='+', help='Object IDs; default: all OBJ filenames in the configured directory')
    assets.add_argument('--output', type=Path, default=Path('outputs/local_trials'))
    verify = sub.add_parser('verify-output', help='Cross-check a completed simulator run and its numeric trajectory records')
    verify.add_argument('--input', type=Path, required=True, help='Directory containing run.json and results.json')
    paper = sub.add_parser("paper-replay", help="Export and verify Table I replay records")
    paper.add_argument("--output", type=Path, default=Path("outputs/paper_replay"))
    paper.add_argument("--seed", type=int, default=0)
    recorded = sub.add_parser("replay-trial", help="Convert a paper replay ID to an Isaac Sim trial")
    recorded.add_argument("--record", required=True)
    recorded.add_argument("--records", type=Path, help="Default: bundled paper replay records")
    recorded.add_argument("--dataset", type=Path, help="Optional local method dataset manifest")
    recorded.add_argument("--output", type=Path, default=Path("outputs/replay_trial.json"))
    pipeline = sub.add_parser("from-pipeline", help="Convert a completed one-command method run")
    pipeline.add_argument("--pipeline", type=Path, required=True)
    pipeline.add_argument("--seed", type=int, default=0)
    pipeline.add_argument("--output", type=Path, default=Path("outputs/pipeline_trial.json"))
    experiment = sub.add_parser("from-experiment", help="Plan paired FS/A1/A2/A3 trials with a fixed receiver")
    experiment.add_argument("--manifest", type=Path, required=True)
    experiment.add_argument("--seed", type=int, default=0)
    experiment.add_argument("--iterations", type=int, default=200)
    experiment.add_argument("--output", type=Path, default=Path("outputs/experiment_trials"))
    for command in ("demo", "evaluate"):
        p = sub.add_parser(command, help="Run Isaac Sim" if command == "demo" else "Check traces without a simulator")
        p.add_argument("--object", choices=[*NAMES, "all"], default="hammer")
        p.add_argument("--variant", choices=[*VARIANTS, "all"], default="intent_aware")
        p.add_argument("--trial", type=Path, help="Custom handover.trial.v1 JSON instead of bundled demos")
        p.add_argument("--trials", type=Path, help="JSON array of custom trials, e.g. from-dataset output")
        p.add_argument("--output", type=Path, default=Path(f"outputs/{command}"))
        if command == "demo":
            p.add_argument("--headless", action="store_true")
            p.add_argument("--hold", action="store_true", help="Keep GUI open after replay")
            p.add_argument("--render-every", type=int, default=1)
            p.add_argument("--screenshot", action="store_true")
            p.add_argument("--animation", action="store_true", help="Export a USD with sampled replay animation")
            p.add_argument("--video", action="store_true", help="Record actual viewport frames to MP4 (requires ffmpeg)")
            p.add_argument("--video-speed", type=float, default=1., help="Playback speed; 0.25 gives quarter-speed review")
            p.add_argument("--camera", choices=["overview", "handover", "left", "right", "top"], default="overview")
            p.add_argument("--visual-style", choices=["lab", "debug"], default="lab", help="Lab lighting/materials or original diagnostic scene")
            p.add_argument("--renderer", choices=["realtime", "pathtraced"], default="realtime", help="Path tracing uses 128 samples per frame for final media")
            p.add_argument("--asset-config", type=Path, help="Local UR5e/Robotiq USD and object mesh configuration")
            p.add_argument("--hand-collision", choices=["boxes", "mesh"], default="boxes",
                           help="Use supplied hand triangles for PhysX safety instead of boxes")
    convert = sub.add_parser("from-intent", help="Convert method scene + selection to a demo joint-replay trial")
    convert.add_argument("--scene", type=Path, required=True)
    convert.add_argument("--selection", type=Path, required=True)
    convert.add_argument("--delivery", type=Path, help="Method delivery target; runs pose IK and RRT-Connect")
    convert.add_argument("--seed", type=int, default=0)
    convert.add_argument("--output", type=Path, default=Path("outputs/intent_trial.json"))
    plan = sub.add_parser("plan", help="Solve the trial target pose and search a collision-checked joint path")
    plan.add_argument("--trial", type=Path, required=True)
    plan.add_argument("--seed", type=int, default=0)
    plan.add_argument("--iterations", type=int, default=600)
    plan.add_argument("--output", type=Path, default=Path("outputs/planned_trial.json"))
    dataset = sub.add_parser("from-dataset", help="Convert a method dataset manifest to offline demo trials")
    dataset.add_argument("--manifest", type=Path, required=True)
    dataset.add_argument("--output", type=Path, default=Path("outputs/dataset_trials"))
    args = parser.parse_args(argv)
    try:
        if args.command == 'review-video':
            from .video import review_montage
            print(review_montage(args.input,args.output))
            return
        if args.command == 'receiver-scenes':
            from .receivers import scene_batch
            manifest=scene_batch(args.scene,args.receiver_config,args.output,args.samples,args.seed)
            print(f"Prepared {len(manifest['scenes'])} receiver scenes for method selection: {(args.output/'scenes.json').resolve()}")
            return
        if args.command == 'prepare-candidates':
            from .candidate_calibration import export
            status=export(args.scene,args.asset_config,args.output)
            print(f"Calibrated {status['contact_valid']}/{status['candidates']} candidates; all IDs retained: {args.output.resolve()}")
            return
        if args.command == 'sample-receivers':
            from .receivers import generate
            trials=generate(json.loads(args.trials.read_text()),args.receiver_config,args.output,args.samples,args.seed)
            print(f"Generated {len(trials)} fixed-receiver trials: {(args.output/'trials.json').resolve()}")
            return
        if args.command == 'verify-output':
            from .verification import verify_output
            report = verify_output(args.input)
            print(f"Verified {report['trials']} trials / {report['frames']} frames: {args.input.resolve()}")
            return
        if args.command == 'doctor':
            from .doctor import inspect_environment
            report = inspect_environment(args.isaac, args.video, args.asset_config)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(report, indent=2))
            for check in report['checks']:
                print(f"[{check['status']}] {check['name']}: {check['detail']}")
            if report['status'] == 'failed': parser.exit(2, 'Required checks failed. See the actions above.\n')
            return
        if args.command == 'from-assets':
            from .asset_demos import generate
            trials = generate(args.asset_config, args.output, args.objects)
            print(f"Generated {len(trials)} local mesh demos: {(args.output/'trials.json').resolve()}")
            return
        if args.command == "paper-replay":
            from .paper_replay import export
            audit = export(args.output, args.seed)
            print(f"Replay: {audit['records']} records, {audit['checked_table_cells']} Table I cells verified: {args.output.resolve()}")
            return
        if args.command == "replay-trial":
            from .paper_replay import read_records, materialize
            record = next((r for r in read_records(args.records) if r['id'] == args.record), None)
            if record is None: raise ValueError(f"Unknown replay record: {args.record}")
            trial = materialize(record, args.dataset)
            validate_trial(trial)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(trial, allow_nan=False))
            print(args.output.resolve())
            return
        if args.command == "from-pipeline":
            from .workflows import from_pipeline
            trial = from_pipeline(args.pipeline, args.seed)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(trial, allow_nan=False))
            print(args.output.resolve())
            if trial.get("planning", {}).get("status") == "failed":
                parser.exit(2, "Planning failed; failed trial saved for evaluation\n")
            return
        if args.command == "from-experiment":
            from .workflows import from_experiment
            trials, state = from_experiment(args.manifest, args.output, args.seed, args.iterations)
            print(f"{len(trials)} paired trials, {state['planning_failures']} planning failures retained: {(args.output/'trials.json').resolve()}")
            return
        if args.command == "from-dataset":
            manifest = json.loads(args.manifest.read_text())
            if manifest.get("schema_version") != "handover.dataset.v1":
                raise ValueError("Expected handover.dataset.v1 manifest")
            root = args.manifest.resolve().parent
            trials, skipped = [], []
            args.output.mkdir(parents=True, exist_ok=True)
            for record in manifest["objects"]:
                paths = [(root/record[key]).resolve() for key in ("scene", "selection")]
                if not all(p.is_relative_to(root) for p in paths):
                    raise ValueError("Dataset input escapes its manifest directory")
                scene, selection = [json.loads(p.read_text()) for p in paths]
                if selection.get("status") != "ok":
                    skipped.append({"object_id": record["object_id"], "reason": selection.get("status")})
                    continue
                trial = from_selection(scene, selection, variant="configured_object_demo")
                validate_trial(trial)
                trials.append(trial)
                (args.output/f"{trial['id']}.json").write_text(json.dumps(trial, allow_nan=False))
            (args.output/"trials.json").write_text(json.dumps(trials, allow_nan=False))
            (args.output/"conversion.json").write_text(json.dumps({"converted": len(trials), "skipped": skipped,
                "source_missing": manifest.get("missing", []), "scope": "configured objects, generated offline hand/grasp demos"}, indent=2))
            print(f"Converted {len(trials)} trials, skipped {len(skipped)}. Batch: {(args.output/'trials.json').resolve()}")
            if not trials:
                parser.exit(2, "No feasible selections to replay\n")
            return
        if args.command in ("from-intent", "plan"):
            if args.command == "from-intent":
                selection=json.loads(args.selection.read_text())
                trial = from_selection(json.loads(args.scene.read_text()), selection,
                    variant=selection.get('mode') or 'intent_aware',
                    delivery=json.loads(args.delivery.read_text()) if args.delivery else None)
                if 'receiver' in trial: trial['id'] += '_' + trial['receiver']['id']
                if trial.get('receiver_protocol',{}).get('replan_in_isaac'):
                    trial['receiver_protocol']['planning_seed']=args.seed
            else:
                trial = json.loads(args.trial.read_text())
            if args.command == "plan" or (args.delivery and not trial.get('receiver_protocol',{}).get('replan_in_isaac')):
                from .planning import plan_trial
                trial = plan_trial(trial, seed=args.seed, iterations=getattr(args, "iterations", 600))
            validate_trial(trial)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(trial, indent=2, allow_nan=False))
            print(args.output.resolve())
            if trial.get("planning", {}).get("status") == "failed":
                parser.exit(2, f"Planning failed: {trial['planning']['reason']}; failed trial saved for evaluation\n")
            return
        if args.trial and args.trials:
            raise ValueError("Choose --trial or --trials, not both")
        trials = (json.loads(args.trials.read_text()) if args.trials else [json.loads(args.trial.read_text())] if args.trial else
                  [load_demo(n, v) for n in (NAMES if args.object == "all" else [args.object])
                   for v in (VARIANTS if args.variant == "all" else [args.variant])])
        if not isinstance(trials, list) or not trials:
            raise ValueError("Expected a nonempty JSON trial array")
        for trial in trials:
            validate_trial(trial)
            import re
            if not re.fullmatch(r"[a-zA-Z0-9_-]+", trial["id"]):
                raise ValueError("Trial id must contain only letters, digits, underscores and hyphens")
        if len({trial['id'] for trial in trials}) != len(trials):
            raise ValueError("Duplicate trial ids are not allowed in a batch")
        args.output.mkdir(parents=True, exist_ok=True)
        if args.command == "demo":
            from .runner import replay
            if args.asset_config:
                from .local_assets import configuration, attach
                trials = attach(trials, configuration(args.asset_config))
            results = replay(trials, args.output, args.headless, args.hold, args.render_every, args.screenshot, args.animation,
                             hand_collision=args.hand_collision, video=args.video, video_speed=args.video_speed, camera=args.camera,
                             visual_style=args.visual_style, renderer=args.renderer)
        else:
            results = [evaluate(t) for t in trials]
            for result in results:
                print(f"{result['trial_id']}: {result['first_failure'] or 'success'} (offline geometry)")
        save_results(results, args.output)
        print(f"Results: {args.output.resolve()}")
    except (ValueError, KeyError, TypeError, AttributeError, OSError, ImportError, RuntimeError) as exc:
        parser.exit(2, f"Error: {exc}\n")
