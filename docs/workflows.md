# Method-to-simulator workflows

Install `python -m pip install -e '.[planning]'` for conversions that generate
new trajectories. The companion method and simulator exchange JSON files and
do not need to share a Python environment.

## Complete neural run

```bash
r2handoversim from-pipeline \
  --pipeline /path/to/intent-handover/outputs/pipeline/pipeline.json \
  --output outputs/neural_trial.json
r2handoversim demo --trial outputs/neural_trial.json --headless \
  --hand-collision mesh --screenshot --animation --output outputs/neural_isaac
```

The manifest must describe a completed method run. Conversion reads its scene,
selection and optional delivery target. A delivery target triggers numerical
IK and planning; a failure saves the failed trial and returns exit code 2.
Without a delivery target, conversion uses the standard stored replay path.

## Paired FS/A1/A2/A3 experiments

First run `intent-handover ablate` in the method environment, optionally with a
dataset `--manifest` or custom `--scene`. Then:

```bash
r2handoversim from-experiment \
  --manifest /path/to/intent-handover/outputs/ablation/experiment.json \
  --seed 0 --iterations 200 --output outputs/ablation_trials
r2handoversim demo --trials outputs/ablation_trials/trials.json --headless \
  --screenshot --render-every 6 --output outputs/ablation_isaac
```

The reference is FS when feasible, otherwise the first feasible mode. Its grasp
places the object at the standard demo endpoint. The common object/hand target
is raised if necessary until the lowest proxy corner is 6 cm above the 0.735 m
demo tabletop. The target is then held fixed across modes; changing the grasp
changes the required robot pose, not the receiver location. Each trial records
the reference and applied height shift. This is a release placement convention.

Each grasp gets its own pose-IK/RRT-Connect plan using the same random seed.
Planning failures remain in `trials.json` and count as failures in evaluation.
`conversion.json` lists selections with no feasible grasp separately: they
cannot be replayed and are excluded from simulator denominators, so inspect
that file along with `report.html` to interpret the evaluated population.

Reports contain per-mode results, first-failure attribution and the existing
object/split aggregation. `evaluate --trials ...` is available for CPU inspection;
`demo --trials ...` is the Isaac Sim execution path.
