# Release validation — 2026-09-30

Executed on Linux, NVIDIA RTX 4070 (12 GB), Isaac Sim 5.0.0, Python 3.11.

## 0.1.0 initial release

- Seven CPU tests passed, covering failure cases, S0 exclusion, failure-order
  attribution, endpoint validation, physics-trace coverage, aggregate-rate
  consistency and forward kinematics.
- All 12 bundled trials ran in headless Isaac Sim, with screenshots and final
  USD scenes exported. PhysX detected the intentionally unsafe trajectories
  (15 contact frames for hammer, 13 for screwdriver, 15 for bottle).
- Normal trials succeeded for all three objects. Missed deliveries failed Reach;
  the region-agnostic screwdriver failed Affordance. Detailed flags are in
  `demo_results.json`, with fixture provenance and evaluation settings.
- A scene/selection pair exported by the separately installed method package
  was converted to a trial and replayed successfully through `scripts/run_isaac.py`.
- Editable installation, standalone wheel build and wheel-installed CPU demos
  were checked outside the source directory. No sibling repository was imported.
- Screenshots were visually inspected. The README image comes from an actual
  Isaac Sim run using the procedural assets.

The local Conda runtime logged missing GCC_12.0.0 symbols in unused RTX sensor
extensions. A second integration run with the process-scoped setting
`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libgcc_s.so.1` completed without those
extension errors. This is a Linux/Conda troubleshooting option only; it is not
embedded in the portable entry point, and no environment libraries were replaced.

Validation used Isaac Sim 5.0 and the kinematic replay protocol.

## 0.2.0 follow-up

- Fourteen CPU tests passed, including input validation, duplicate IDs, stale
  completion manifests and artifact links, and nonzero worker exits.
- All 12 bundled trials ran again in Isaac Sim. Every metric, first failure and
  contact-frame count matched `demo_results.json` from the initial release.
- A real H2O Text2HOI prediction decoded by the companion method with locally
  supplied MANO ran through `from-intent` and Isaac Sim. The decoded mesh was
  rendered, its skeleton proxies were evaluated, and the bottle trial passed
  all active criteria. The screenshot was visually inspected; generated MANO
  assets are excluded from Git.
- HTML reports, JSON/CSV results, final USD snapshots and animated USD exports
  were generated successfully.
- A deliberately blocked USD export raised an internal Kit exception while Kit
  returned exit code 0. The supervising CLI correctly returned exit code 2 and
  retained a failed run manifest. A missing Isaac Sim installation also returned
  exit code 2 with installation guidance.
- Reopened the animated USD through Isaac Sim's USD API: every robot part has
  90 time samples, transforms differ between first and last frames, and the
  timeline is configured at 60 frames per second.
- Version 0.2.0 wheel installed independently; all 12 CPU evaluations ran from
  outside the source directory and wrote the standalone HTML report.

## 0.3.0 paper-detail follow-up

- Twenty CPU tests pass. New coverage includes full-pose IK, RRT detouring around
  a blocking obstacle, attached-object collision, unreachable-target failure,
  mesh-mode input requirements, and unbalanced object/split aggregation.
- The skeletal ergonomic target produced by the companion method yielded an
  85-frame planned trajectory. Isaac Sim replay passed all five S1 criteria and
  exported a screenshot, report and animation. Supplied keypoint visualization
  was visually inspected using the screenshot exported by that run.
- The real coarse Text2HOI/MANO bottle output ran with triangle-mesh hand
  collision enabled and passed all active S0 criteria.
- A separate unsafe trajectory with a procedural triangulated hand generated
  15 contact frames and a Safe failure, confirming mesh colliders participate
  in PhysX queries rather than merely being rendered.
- All 12 original bundled traces ran again in Isaac Sim; metrics, first-failure
  labels and contact counts matched the recorded initial release results.
- Version 0.3.0 standalone wheels built successfully. Independently installed
  method delivery, benchmark IK/planning and offline evaluation ran outside
  both source directories.

## 0.4.0 configured-data follow-up

- Twenty-two CPU tests pass, including source point-cloud/split preservation and
  malformed cloud rejection.
- All 16 configured han object demos completed in Isaac Sim 5.0, each exporting
  its original point-cloud display, result and screenshot. All active demo S0
  criteria passed using the generated fixture annotations and trajectories.
- The actual binoculars cloud plus a left-hand Text2HOI/MANO prediction ran in
  triangle-hand-collision mode and passed all active criteria. Its screenshot
  was visually inspected. The animated USD was exported.
- Standalone 0.4.0 wheels were built, installed independently, and exercised
  through config import, dataset batch conversion and offline evaluation.
- Reopened the neural-object animation in Isaac Sim's USD API: all 8192 source
  points were retained and the object cloud had 90 animated transform samples.

## 0.5.0 runnable-workflow follow-up

- Twenty-five tests pass, including shared receiver/object placement across
  different grasps, preservation of failed plans, incomplete pipeline rejection
  and duplicate-ID rejection in offline evaluation.
- Converted the method's three-object/four-mode experiment with seed 0 and 80
  RRT iterations. All 12 trials completed Isaac Sim replay and screenshot export:
  nine passed, two failed Affordance, one failed Plan. These are demo outcomes.
- Enabling table-aware planning exposed the old fixed-target placement's table
  intersections. The experiment converter now applies one shared clearance lift
  per object before planning any mode, preserving the paired receiver geometry.
- A fresh original-weight bottle pipeline ran through manifest conversion and
  triangle-mesh-hand PhysX evaluation. All active S0 criteria passed. Screenshot,
  final USD, animated USD and reports were exported; screenshots were inspected.
- Built and independently installed the 0.5.0 wheel and exercised manifest
  conversion, paired planning and CPU evaluation outside both source trees.

## 0.6.0 viewport recording

- Twenty-nine benchmark tests pass (plus 23 unchanged method tests). New tests
  cover missing encoders, partial PNG writes, missing frames and encoder failures
  that must preserve an existing video.
- Actual Isaac Sim viewport recordings exercised the planned ergonomic hammer,
  coarse Text2HOI/MANO bottle, configured binoculars point cloud with a predicted
  left hand, an affordance failure and an execution-deviation safety failure.
- MP4 capture renders every executed frame and waits for PNG completion before
  encoding. The bottle run captured 90 simulation frames and produced a 1280x800,
  H.264, 30 fps video at quarter speed, with start/end inspection holds.
- Recording preserves the failure classifications; the unsafe hammer still
  produces exactly 15 PhysX hand-contact frames. Capture overhead is included
  only in simulator wall time, not trajectory duration.
- Generated MANO-derived videos remain local review artifacts, outside Git.

## Original-asset replay (0.7.0)

Validated locally with Isaac Sim 5.0.0 on RTX 4070:

- Original `danilab_ur5e` USD: 36 mesh prims, 16 robot collider prims, UR5e six-joint motion and Robotiq 2F-85 fingers.
- All 16 configured OBJ objects loaded and completed viewport MP4, screenshot, and time-sampled USD export. This validates asset loading and replay export.
- A Text2HOI-predicted MANO hand and original binocular mesh replayed with triangle-mesh hand collision.
- Six authored reference-outcome examples (success, Plan, Reach, Safe, Stability, Affordance) were retargeted for the original tool and table clearance. Independent simulator predicates matched all six; the Safe example had six detected contact frames.
- 8,000 bundled reconstructed records match 108 Table I cells. Wheel resource loading and deterministic regeneration checked. These aggregate checks concern reference records, separately from the six simulator examples.
- 35 CPU tests pass. Optional local assets are not part of CI or redistributed in the wheel.

## Bilateral contact correction (0.7.1)

The first asset replay exposed grasps that followed a tool transform while the
pads did not reach the object. The asset adapter now fits a shallow insertion
and a local opposing surface pair, calibrates the nonlinear Robotiq opening,
and checks each contact against the original finger collider triangles.
All 16 local object meshes passed the 0.2 mm bilateral-distance threshold.
The report retains the original grasp and exposes both measured distances.
The handover camera now includes the grasp location, including for long objects.
38 CPU tests pass, including detached approaches and objects exceeding aperture.
This validates geometric contact placement during kinematic replay.

## Resolved trajectory exports (0.8.0)

- 44 CPU tests pass, including scene preparation idempotence, changed-asset rejection, receiver/skeleton frame consistency and separation of reference labels from measured results.
- A real-asset binocular/MANO replay was exported and reloaded in Isaac Sim. All 90 timestamps, joint states, tool/object poses and PhysX contact observations agreed to an absolute tolerance of 1e-10.
- The six authored outcome examples were replayed again. All exported NPZ contact frames and durations agree with the JSON results, and all six reference outcomes match the evaluated outcomes.
- Resolved JSON embeds the scene actually used, including calibration and receiver retargeting. The NPZ stores numeric frame observations without pickle objects. This validates the kinematic replay exports.

## Public release checks (0.9.0)

- 51 CPU tests pass, including the standalone local-mesh generator, missing
  environment/encoder checks, malformed CLI input, calibrated-tool planning,
  and detection of inconsistent exported collision observations.
- A fresh Python 3.10 virtual environment installed only the built wheel with
  planning/assets extras (NumPy 2.2.6, SciPy 1.15.3, trimesh 5.1.0, Pillow
  12.3.0). The tests and installed-package smoke passed. All 12 offline fixtures,
  8,000 replay records / 108 reference cells, and record materialization ran
  outside source-package imports. Wheel/source metadata checks passed, and the
  source archive rebuilt a wheel independently.
- The wheel installed into a separate directory ran all 12 default Isaac Sim
  fixtures from `/tmp`; metrics, first failures and contact-frame counts matched
  the stored regression results. Screenshots, animations and numeric trajectory
  exports were produced and cross-checked.
- The new `from-assets` command generated all 16 local OBJ demos without reading
  the method repository. All 16 completed original UR5e/Robotiq replay and video,
  image, USD animation and JSON/NPZ export: 1,440 frames and 96 output artifacts.
  Maximum bilateral pad-surface distance was 0.001651 mm (threshold 0.2 mm).
  Maximum observed-tool versus model position difference was 3.06e-8 m.
- A 90-frame resolved bottle scene reloaded without refitting; timestamps,
  joints, tool/object poses, grasp and contact observations agree to 1e-10.
  The final wheel also replayed and recorded that real-asset scene independently.
- `verify-output` passed for the 16-object run, 12 default fixtures, six authored
  reference outcomes and the existing 90-frame MANO/binocular scene. The six
  outcome examples retain separate assigned reference and evaluated labels.
- Source archives include the configuration template, launcher, tests and docs;
  release archives exclude local robot USDs, original OBJ/PLY assets, weights,
  MANO models and generated private output directories. CI repeats the package
  build/install/smoke checks across Python 3.10–3.12.

The 16 generated scenes validate release functionality using authored
hand proxies and geometric approaches. The 73-second review montage contains actual
Isaac Sim viewport captures with object/setting/outcome labels.

## 0.10.0 fixed-world hand and method integration

Original UR5e/Robotiq USD, original OBJ objects and visible left/right MANO meshes
were used in the live Isaac Sim tests. Each object's four bounded SE(3) samples
(seed 27) were fixed before FS/A1/A2/A3 selection. The method used the explicit
`object_projection` feasibility policy; the local pad aperture was preserved
separately. Numerical records and method post-audits both passed.

| Local setting | FS | A1 | A2 | A3 | Recorded frames |
|---|---:|---:|---:|---:|---:|
| Can, S0, successes / 4 trials | 3 | 1 | 4 | 4 | 1,582 |
| Screwdriver, authored S1, successes / 4 trials | 4 | 0 | 4 | 4 | 2,518 |

Can failures were Plan failures. Screwdriver A1 failed Plan twice and Affordance
twice. All 32 resolved trials preserved selected grasp, local opening, receiver,
object target and full-projection width. The table reports these local
samples under the stated settings and authored region annotations.

A separate original-weight Text2HOI 1000-step → MANO → selection → authored
skeletal delivery route reached the benchmark. Its one trial failed Plan with
eight IK solutions and collision rejections (object/hand, self and environment).
The predicted hand and original delivery target were kept; the failure was not
retargeted into a success. A separate live-USD positive control placed a small
usage region on a finger-pad contact: only Affordance failed, confirming the
query can detect intrusion. These two runs are not pooled into the table above.

The new handover camera chooses a receiver view using original USD world bounds
at the start, middle and end of the saved trajectory. A formerly occluded S1
pose was rechecked visually with the hand and both grasping geometry and robot
visible. Camera-only recordings retain the original numeric trials, avoiding
changes to receiver poses or measured outcomes to improve a movie.

The neutral hand exporter was tested without importing the companion repository;
its 778-vertex/1,538-face left/right rest templates matched the locally decoded
zero-pose templates within 2e-8 m. Model files and generated meshes remain local.
Per-trial results, frame counts and scene poses are in
[fixed_receiver_validation.json](fixed_receiver_validation.json).

The mixed-object recording stress test exposed a stale PhysX scene during
same-path USD replacement. The backend now creates a fresh USD/PhysX world per
trial, verifies eight small surface probes on the current hand, and checkpoints
completed media metadata. All 33 original method inputs were then **replanned**
in fresh scenes: geometry, joint trajectories, five metrics and first failures
were unchanged, and all 264 hand-surface probes passed. The interrupted movie
batch remains marked failed; it is not treated as a complete 33-trial run.

The paper-aligned receiver config now defaults to reference-IK-conditioned
sampling, before method selection. A separate can run used two accepted proposals
(seed 27; no rejected proposals in this small draw), then four method modes on
each fixed scene: eight trials passed export verification, with five successes
and three Plan failures. Reference IK defines the sampling set and does not
promise that every subsequently selected grasp has a feasible Plan. This is an
explicit reference-IK-conditioned reachable-set definition. Earlier bounded-workspace results retain their
original labels and are not retroactively called conditioned samples.


## Outward receiver region (after 0.10.0)

The default region was moved farther from the supported robot base in response
to review of the camera replay. Palm world bounds are now X [-0.72, -0.60],
Y [-0.45, -0.30], Z [0.88, 0.98] m (horizontal base distance 0.67–0.85 m).
The old 33-case records and published 0.10.0 archives keep their original settings.

A new seed-27 draw used two receiver poses per object, alternating left/right,
with reference IK enabled. Can accepted two of four proposals; screwdriver
accepted both proposals. The four accepted palm distances were 0.708, 0.780,
0.731 and 0.743 m. All method selections were recomputed on these new scenes.
Actual Isaac Sim FS replay completed four records / 166 frames: screwdriver
left succeeded; both can poses and screwdriver right failed to find IK for the
selected FS grasp. All four passed output verification and retain the sampled
hand and object target. These records support the distance/scene review. Reference-grasp IK conditioning does not guarantee method-grasp IK.

## Laboratory render preset (after 0.10.0)

The grid floor/embedded point light were replaced visually by grey laboratory
surfaces and five area lights. The room walls have no collision APIs. Robot
materials, original object colors, physical vertices and collider triangles
are preserved. Lab presentation hides Reach/skeleton guides, not metric data.

One resolved 163-frame original-asset screwdriver trajectory was recorded from
four cameras using path tracing (128 samples, six bounces, temporal denoising
disabled). Each export passed `verify-output`; every NPZ array, metric,
selection, receiver pose and contact record matched the previous recording.
These are four camera views of the same replay.
The 12 default fixtures also passed a fresh realtime lab-preset run (1,077
frames), with all numeric arrays and results unchanged from the earlier
fixture regression. The CPU suite passed all 64 tests and the wheel built.

## README original-asset showcase — 2026-10-01

Recorded four A2 scenes in separate Isaac Sim 5.0 processes with the lab preset,
path tracing and the right camera: can left/right (169/91 frames), screwdriver
left/right (247/301 frames). All four runs passed `verify-output`, totalling
808 observed frames, and all applicable metric checks passed. Receiver seed 27
and outward sampling bounds match the configured fixed-world scene inputs.

The homepage presents these four independent 0.5× replays as a labelled montage,
and the previously verified FS screwdriver replay as synchronized dual-view and
four-camera media. Inline GIFs link to HD MP4 attachments. The
[media record](media/recordings.json) retains each displayed trial's evaluated
outcome and method setting.

## Posture-reviewed showcase — 2026-10-01

The current hero is the seed-41 screwdriver right-receiver sample 5, selected
with FS and evaluated in Isaac Sim. Its endpoint joint angles are
`[18.05, -51.89, 55.70, -84.29, -140.87, 6.31]` degrees: a bent elbow and a small
terminal wrist roll. All five checks passed across 91 executed frames.
Overview, left, right and elevated recordings passed `verify-output` and have
identical trajectory arrays and evaluated metrics.

The A2 gallery uses can samples 10/5 and screwdriver samples 0/21 (left/right),
all from the same seed-41 receiver bank and configured outward bounds. The
four trajectories contain 79, 97, 169 and 127 frames; all applicable checks
passed. The full-arm camera views replay those resolved scenes. Each method
selection was recomputed on its sampled scene before simulation. Media sample
IDs, endpoint angles and outcomes are retained in [recordings.json](media/recordings.json).
