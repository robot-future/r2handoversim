# Release 0.11.0

This release is ready for users to install and run kinematic handover replay.
Its supported workflows are procedural Isaac Sim demos, original local
UR5e/Robotiq and OBJ replay, method JSON interchange, and paper-reference replay.
The main benchmark entry point runs Isaac Sim; CPU evaluation is auxiliary.

## New in 0.11.0

The default receiver region is farther from the supported robot base (0.67–0.85 m
horizontal palm distance), while retaining preselection reference-IK filtering.
The default lab appearance uses neutral grey surfaces, a dark workbench and five
soft area lights. Optional path tracing and left/right/elevated cameras produce
complementary review videos. See [rendering](rendering.md) for commands.

The four-camera recording and all 12 default fixtures were checked against the
previous numeric exports: trajectories, contacts and evaluated results were
unchanged. Existing saved trials keep their original receiver positions.

## Fixed receiver integration

The release includes visible sampled MANO meshes, immutable world hand/object
poses across method modes, preselection candidate contact calibration, and
original-collider PhysX planning. Follow [fixed receivers](fixed_receivers.md)
for the supported sequence and configuration details.

## Release gates

- CPU suite exercises geometry, failure attribution, planning, converters,
  input rejection, recording failures, contact fitting and export consistency.
- CI builds both wheel and source distribution on Python 3.10, 3.11 and 3.12,
  checks package metadata, installs the wheel, runs tests outside package source
  imports, exercises all 12 offline fixtures and the 8,000-record paper replay,
  then rebuilds a wheel from the source archive.
- The actual GPU smoke check uses Isaac Sim 5.0 on Linux. It covers the 12 default
  fixtures, 16 original object meshes with UR5e/Robotiq, frame capture, animated
  USD, resolved JSON/NPZ export, and export/reload consistency. This local check
  is separate from CI, which has neither the simulator nor private assets.
- Every original-asset successful fit verifies both object contacts against the
  USD finger-pad surfaces with a 0.2 mm threshold. Failure-case examples remain
  separately labeled and checked against evaluated outcomes.
- `verify-output` checks completed-run counts, artifact presence, numeric
  trajectories and result consistency. Videos are rendered by Isaac Sim.
- Package files contain no local model weights, robot USDs, dataset meshes,
  MANO assets or participant records. External assets retain their source terms.

See [validation](validation.md) for executed checks and scope. A completed run
requires a successful `run.json`; a nonzero CLI exit or failed manifest must not
be treated as a measured benchmark success.

## Maintainer commands

Run tests in an environment with the optional `planning,assets` dependencies.

```bash
python -m pip install build twine
python -m build
python -m twine check dist/*
python -m pip install './dist/r2handoversim-0.11.0-py3-none-any.whl[planning,assets]'
python -m unittest discover -s tests -v
python scripts/release_smoke.py
```

For the simulator, follow the README's default and local-asset commands, then
run `verify-output` on each output directory. Inspect the movies together with
the recorded bilateral pad-contact distances. Keep private generated artifacts
out of source releases.

Versioned wheel and source archives are the distribution artifacts. A GitHub
release can attach those two files and their SHA-256 checksums. A source checkout
also works with `scripts/run_isaac.py` in an installed Isaac Sim environment.

## Supported workflows and inputs

The four baseline settings use offline authored/reconstructed inputs. Table I
source values and reconstructed aggregates have separate records from evaluated
replay outcomes. The robot follows kinematic joint trajectories with a rigidly
attached object. The procedural CPU planner uses
sampled box proxies; fixed receiver scenes use the original-collider PhysX
planner. The legacy asset-demo retargeting workflow changes the receiving hand
during fitting. Use `receiver-scenes` before method selection for fixed-world
comparisons.

The original asset adapter uses the documented `danilab_ur5e` assembly and its
frames, joints and colliders. The bundled procedural demos provide a
self-contained starting point immediately after installing Isaac Sim.
