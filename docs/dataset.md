# Replay locally configured objects

This page describes the original point-cloud importer. Original OBJ meshes and
preselection candidate/receiver scenes now use the
[fixed receiver workflow](fixed_receivers.md); that workflow enables original
collider planning and hull-based metrics.

The companion method reads the original Text2HOI YAML config and emits a
`handover.dataset.v1` manifest. This benchmark consumes those exported JSON
files; it neither imports the companion Python package nor loads the legacy
pickle itself.

```bash
r2handoversim from-dataset \
  --manifest /path/to/intent-handover/outputs/han_dataset/dataset.json \
  --output outputs/han_trials
r2handoversim demo --trials outputs/han_trials/trials.json --headless \
  --screenshot --render-every 12 --output outputs/han_isaac
```

Conversion writes individual `*_configured_object_demo.json` trials plus the
`trials.json` array. `conversion.json` records infeasible selections skipped
and missing original assets. All 16 available han objects converted and ran in
Isaac Sim 5.0 on the validation machine. Offline inspection also accepts
`evaluate --trials outputs/han_trials/trials.json`.

`object_points_object` stores the original full-resolution cloud in canonical
object coordinates. Isaac Sim renders it as USD Points, transforms it by
`T_world_gripper @ inverse(T_object_gripper)` each frame, and hides the object
proxy visuals. `--animation` saves the same object-cloud transform samples.
Metrics use `object_boxes` for collision, width and reach in this point-cloud workflow.

The 16 PLY files provide object points for display and neural input. Robot
and hand geometry use the configured proxies or a supplied decoded MANO mesh. Source data paths/hashes and annotation status
are preserved in trials and results.

The bootstrap hand, 30 grasp candidates, receiving-zone labels and trajectories
are generated offline fixtures. Imported examples select S0 explicitly and
record their generated annotation provenance. Table I reference records are
available through the separate `paper-replay` workflow.

## Real object plus predicted hand

After the method's binoculars Text2HOI/MANO example:

```bash
r2handoversim from-intent \
  --scene /path/to/intent-handover/outputs/han_predicted/binoculars_scene.json \
  --selection /path/to/intent-handover/outputs/han_predicted/binoculars_FS.json \
  --output outputs/han_neural_trial.json
r2handoversim demo --trial outputs/han_neural_trial.json --headless \
  --hand-collision mesh --screenshot --animation --output outputs/han_neural
```

This displays the original object point cloud alongside the decoded hand and
uses the static hand triangles for PhysX Safe evaluation. Generated assets and
screenshots stay in ignored local output folders; no local dataset or MANO
geometry is added to the source release.
