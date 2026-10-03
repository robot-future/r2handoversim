# Start with an original-asset scene

This guide produces a UR5e/Robotiq replay with an object mesh and a fixed,
visible left or right receiving hand. Run commands from the benchmark root.

## 1 · Install

Use the Python environment supplied by **Isaac Sim 5.0**. Install `ffmpeg` for MP4 recording.

```bash
python -m pip install -e '.[planning,assets]'
r2handoversim doctor --isaac --video
```

For installations using `python.sh`, replace `r2handoversim` with
`/path/to/isaac-sim/python.sh scripts/run_isaac.py` throughout this guide.
See [installation help](installation.md) for environment troubleshooting.

<a id="configure-assets"></a>

## 2 · Configure your assets

| Input | Location / convention |
|---|---|
| Robot | Complete `danilab_ur5e` UR5e + Robotiq 2F-85 USD assembly and referenced layers |
| Objects | Original OBJ meshes in metres; names match the scene's object ID |
| Receiver | Licensed left/right MANO templates, or hand OBJ meshes with measured palm frames |
| Method scene | Intent-Handover scene JSON with object geometry, candidate grasps and usage regions |

```bash
mkdir -p outputs
cp configs/local_assets.example.json outputs/local_assets.json
cp configs/receivers.example.json outputs/receivers.json
```

Edit `robot_usd` and `object_mesh_root` in `outputs/local_assets.json`. Keep the
example placement for the documented robot assembly. In `outputs/receivers.json`,
set each template's `mesh` path and matching palm frame. Paths resolve relative
to the configuration file; absolute paths also work.

To create the hand templates from local MANO models, follow the
[template export commands](fixed_receivers.md#inputs-and-ordering). The exporter
writes `outputs/local_hands/config.json`, which can be used directly in place
of `outputs/receivers.json`. The supplied receiver config enables reference-IK
filtering and outward palm sampling.

```bash
r2handoversim doctor --isaac --video --asset-config outputs/local_assets.json
```

Prepare a scene using the companion method's
[local dataset and candidate importer](https://github.com/Hanxin-Zhang/intent-handover/blob/main/docs/dataset.md#restore-locally-available-original-candidates).
Save its scene as `outputs/input_scene.json`. Its object frame and units must
match the configured OBJ. The following commands calibrate the entire candidate
set before selection.

## 3 · Prepare and sample

```bash
r2handoversim prepare-candidates --scene outputs/input_scene.json \
  --asset-config outputs/local_assets.json --output outputs/calibrated_scene.json
r2handoversim receiver-scenes --scene outputs/calibrated_scene.json \
  --receiver-config outputs/receivers.json --samples 4 --seed 27 \
  --output outputs/receiver_scenes
```

The output `scenes.json` lists four independently sampled scenes. Start with
the first one; this small helper copies it to a predictable filename:

```bash
python - <<'PY'
import json
from pathlib import Path
root = Path('outputs/receiver_scenes')
manifest = json.loads((root / 'scenes.json').read_text())
Path('outputs/sampled_scene.json').write_bytes(
    (root / manifest['scenes'][0]['scene']).read_bytes())
PY
```

## 4 · Select and convert

Use the companion [Intent-Handover](https://github.com/Hanxin-Zhang/intent-handover)
installation to run four offline selection settings. If it uses a separate
Python environment, pass absolute input/output paths to this command:

```bash
intent-handover ablate --scene outputs/sampled_scene.json \
  --feasibility-width-policy object_projection --output outputs/method
```

Back in the Isaac Sim environment:

```bash
r2handoversim from-experiment --manifest outputs/method/experiment.json \
  --seed 27 --output outputs/trials
```

`outputs/trials/trials.json` carries each selected grasp and the shared receiver
and object target. Repeat this step for other entries in `scenes.json` to cover
the receiver bank. `conversion.json` records the converted selection population.

## 5 · Record and inspect

```bash
r2handoversim demo --trials outputs/trials/trials.json --headless \
  --visual-style lab --renderer pathtraced --camera right \
  --video --screenshot --animation --output outputs/replay
r2handoversim verify-output --input outputs/replay
r2handoversim review-video --input outputs/replay --output outputs/review.mp4
```

Open `outputs/replay/report.html`. Each trial links its MP4, screenshot, resolved
scene and trajectory NPZ. Metric outcomes and planning diagnostics accompany
both successful and failed trials.

For fast iteration choose `--renderer realtime`. For another view, pass an
exported `*_trial.json` to `demo --trial`, choose `overview`, `left`, `right`,
`top` or `handover`, and use a new output directory. The
[rendering guide](rendering.md) gives the multi-camera commands.
