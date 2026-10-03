# Installation and troubleshooting

## Supported paths

The simulator integration has been exercised on Linux with Isaac Sim 5.0.0,
Python 3.11 and an RTX 4070. Offline tools require Python 3.10 or newer; CI covers
3.10–3.12. Install Isaac Sim separately using NVIDIA's instructions, and accept
its license yourself before launching the demo.

In the Isaac Sim Python environment, install the checkout with:

```bash
python -m pip install -e '.[planning,assets]'
python -m r2handoversim doctor --isaac --video
python -m r2handoversim demo --headless --object bottle --screenshot
```

The `assets` extra installs trimesh and Pillow. `planning` installs SciPy;
`from-assets` uses it to solve the authored demonstration endpoint. The default
procedural replay needs only NumPy in addition to Isaac Sim. Do not upgrade an
existing simulator's packages indiscriminately. The validated local versions
were NumPy 1.26.0, SciPy 1.15.3, trimesh 4.5.1 and Pillow 11.2.1.

For a standalone installation, use its launcher in place of `python`:

```bash
/path/to/isaac-sim/python.sh -m pip install -e '.[planning,assets]'
/path/to/isaac-sim/python.sh scripts/run_isaac.py doctor --isaac
/path/to/isaac-sim/python.sh scripts/run_isaac.py demo --headless --object bottle
```

You can also install a release wheel with `python -m pip install
'./r2handoversim-0.9.0-py3-none-any.whl[planning,assets]'`. Run from any directory
using `python -m r2handoversim` or the `r2handoversim` console command. A source
archive includes the launcher, configuration template, docs and tests.

## Local assets

Copy `configs/local_assets.example.json` and set its two paths. Relative paths
resolve against the configuration file's directory. `robot_usd` must be the
supported `danilab_ur5e` UR5e/Robotiq assembly, including all its referenced
layers. The adapter is not a generic URDF/USD importer. `robot_translation` is
in meters; the template aligns the original assembly to the nominal demo base.

OBJ vertices must already be in meters and share the object frame used by any
imported trial. Keep materials/textures alongside the original OBJ files.
The generator does not infer unit scale. Do not commit private asset configs
or generated mesh/hand outputs; `outputs/` is ignored by default.

## Common errors

| Symptom | Action |
|---|---|
| Isaac Sim import is unavailable | Run `doctor --isaac` with the same Python interpreter as `demo`; use Isaac Sim's Python or `python.sh`. |
| EULA prompt in a headless job | Review and accept NVIDIA's terms using the simulator's documented setup before replay. |
| `ffmpeg` missing / libx264 missing | Install an FFmpeg executable with the libx264 encoder, put it on PATH, then rerun `doctor --video`. |
| Missing robot links, joints or pad colliders | Restore the complete supported USD assembly and referenced layers. A generic UR5e USD is not interchangeable with this adapter. |
| No opposing surfaces fit the aperture | Inspect object scale and approach; provide a suitable explicit trial. The runner does not silently invent a contact. |
| Resolved scene rejects changed assets | Regenerate from the original input with the updated asset configuration. |
| `GCC_12.0.0` symbol errors in Linux/Conda RTX extensions | On a system with a compatible GCC runtime, a process-scoped `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libgcc_s.so.1` resolved this local issue. This path is platform-specific; do not replace environment libraries. |
| Nonzero exit with incomplete results | Read `run.json` and the terminal log. Only `status: succeeded` with the expected trial count confirms a complete simulator run. |

`--hold` is for interactive GUI sessions. Use `--headless` without `--hold` for
batch jobs. MP4 capture waits for every rendered frame, so recording is slower
than replay without recording. Metrics use simulation time, not recording time.
