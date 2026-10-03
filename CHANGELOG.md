# Changelog

## 0.11.0

- Add a neutral laboratory render preset with five area lights, smooth hand
  shading, optional path tracing and complementary detail camera views.

- Move default receiver palm bounds outward to 0.67–0.85 m horizontal distance
  from the supported robot base; retain preselection reference-IK filtering.

## 0.10.0

- Sample visible left/right hand meshes with deterministic SE(3) poses before
  method selection. Preserve receiver and object targets across all modes.
- Calibrate full candidate sets against original USD pads; preserve failed
  candidates and source provenance, and reject post-selection refitting.
- Plan fixed receiver scenes in Isaac Sim using original robot colliders,
  object convex hull and static hand triangles. Retain failed trials at home.
- Add full-mesh Stability gate, hull Reach, original finger/region Affordance,
  per-frame hand contact observations and receiver-pose trajectory exports.
- Fix paired conversion and delivery inputs overwriting fixed receiver targets.
- Add verified review-video captions and explicit paper-fidelity documentation.
- Start a fresh PhysX scene per trial and verify receiver surface positions to
  prevent stale collider queries in mixed-object batches.
- Default public receiver configs to preselection reference-IK conditioning.


## 0.9.0

- Generate local-mesh demo batches without a method checkout, dataset manifest or weights.
- Add environment/encoder/asset preflight checks and readable malformed-input errors.
- Cross-check exported scene/trajectory/result consistency with `verify-output`.
- Correct IK targets and attached-object collision checks for calibrated tool offsets.
- Validate required USD links, joints, pad colliders and reference composition.
- Verify wheel and source distributions from an independent environment; include installation guidance and external-asset terms.


## 0.8.0

- Export resolved replay trials and per-frame tool/object poses, joint positions, timestamps and PhysX hand-overlap observations.
- Reload resolved asset scenes without applying contact fitting or receiver retargeting twice; detect changed assets and calibration.
- Keep hand, skeleton and delivery annotations in the same retargeted frame, and make the changed receiver policy explicit for paired experiments.
- Show reference outcomes, evaluated outcomes and right/left pad distances separately in reports and CSV exports.

## 0.7.1

- Correct detached local-asset grasps by fitting opposing mesh contact surfaces and insertion depth, calibrating actual Robotiq pad spacing, and verifying both contacts against USD collider triangles.
- Report contact distances and preserve the original grasp before replay fitting. Reject missing contacts rather than attaching an ungrasped object.

## 0.7.0

- Add configurable original UR5e/Robotiq USD and local OBJ meshes, calibrated tool frames, actual robot collider queries, and USD/MP4 replay exports.
- Add 8,000 deterministic reconstructed Table I replay records, source settings, exact aggregate verification and independent simulator evaluation.

## 0.6.0

- Record actual Isaac Sim viewport frames to H.264 MP4 with configurable playback
  speed, overview/receiving-hand cameras and report links.
- Wait for asynchronous PNG writes to finish after the viewport capture callback.
- Reject missing encoders/frames and preserve prior videos on encoding failure.

## 0.5.0

- Import a completed method pipeline directly, including optional delivery planning.
- Convert paired FS/A1/A2/A3 experiments while holding the receiving hand and
  object target fixed across modes, with shared tabletop clearance adjustment.
- Retain failed plans in batch evaluation and record infeasible selections separately.
- Reject duplicate trial IDs consistently in offline and Isaac Sim batch runs.

## 0.4.0

- Convert imported dataset manifests to batch trials and accept `--trials` arrays.
- Render original object point clouds in Isaac Sim and animate them with the grasp.
- Preserve source hashes, annotation status and explicitly supplied split labels.
- Validate all 16 locally configured objects and a real-cloud Text2HOI example.

## 0.3.0

- Add numerical full-pose IK and a portable RRT-Connect planner, with sampled
  robot, attached-object, receiving-hand and obstacle box collision checks.
- Consume ergonomic delivery targets without relocating the hand to a fixed goal.
- Add optional static triangle-mesh hand collisions in Isaac Sim.
- Export Table I object/split aggregation and measured planning times.
- Visualize supplied skeletal keypoints and target direction in Isaac Sim.

## 0.2.0

- Supervise Isaac Sim in a separate process and reject stale/incomplete run
  manifests even when Kit exits with code zero.
- Display decoded MANO meshes imported from the method while retaining explicit
  skeletal proxy collision evaluation and the predicted palm normal.
- Add self-contained result reports and optional time-sampled USD animation.
- Validate mesh indices, identifiers, duplicate trials and boolean contact traces.
- Bound screenshot capture time and check USD export success.

## 0.1.0

- Initial Isaac Sim replay examples, PhysX overlap evaluation, offline geometry
  checks and JSON/CSV results.
