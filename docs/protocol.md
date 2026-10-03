# Handover protocol

The original-asset workflow connects UR5e/Robotiq geometry, object meshes and
static receiving-hand meshes in Isaac Sim. Lengths are metres, angles radians;
coordinates are right-handed with Z up. Homogeneous transforms are row-major
JSON arrays acting on column vectors.

## Scene → selection → execution

1. **Prepare candidates.** Calibrate each grasp against the original finger-pad
   surfaces. Preserve candidate IDs, object frame and calibration evidence.
2. **Sample receivers.** Draw a seeded left/right hand pose within configured
   SE(3) bounds. Reference-pose IK filters proposals before method selection.
3. **Select grasps.** Run each method on the same receiver and object target.
   The benchmark Stability policy uses full-object closing-axis projection;
   calibrated local pad aperture separately controls gripper geometry.
4. **Plan and replay.** Solve the full tool pose, then run RRT-Connect using
   original USD robot colliders, the object convex hull and hand triangles.
   Execute kinematic robot motion with rigid object attachment.
5. **Evaluate and export.** Record all metric flags, first failure, numeric
   trajectories, receiver transforms and rendered media.

Use [the quickstart](quickstart.md) for commands and
[receiver configuration](fixed_receivers.md) for frames and sampling settings.

## Evaluation

| Metric | Original-asset evaluation |
|---|---|
| Stability | Full object-mesh projection along the closing axis ≤ 0.085 m |
| Plan | Feasible full-pose IK and collision-checked path |
| Reach | Object hull intersects a sphere centred at palm + 0.12 m × palm normal, radius 0.10 m |
| Affordance | Original finger colliders preserve the supplied human-usage volumes; applies in S1 |
| Safe | Original robot colliders are clear of the static hand mesh at every replay frame |

First-failure attribution is **Stability → Plan → Reach → Affordance → Safe**.
Each record retains all independently evaluated flags. S0 reports Affordance as
null and uses the other four checks for success. Per-split success and
first-failure rates sum to 100%.

Planning uses explicit allowed contacts for internal gripper links, neighbouring
arm links, the mounted shoulder/table and the held object with gripper/distal
wrist. Candidate contact validation checks both pad surfaces at 0.2 mm tolerance.
The default path is sampled at 60 Hz with a 0.6 rad/s joint-speed cap.

## Trial schema

Every `handover.trial.v1` JSON is a replay input. A resolved `*_trial.json`
records the calibrated scene and can be loaded directly with `demo --trial`.

| Field | Meaning |
|---|---|
| `id`, `object_id`, `variant`, `split` | Trial identity, method setting and S0/S1 evaluation setting |
| `provenance`, `source_data` | Input sources and hashes |
| `asset_robot`, `object_mesh_object` | Local robot assembly and object-frame mesh |
| `T_object_gripper`, `grasp_contract` | Selected grasp and frame/width conventions |
| `T_tcp_asset_tool` | Calibrated robot-tool transform |
| `target_T_world_object` | Shared object delivery pose |
| `target_T_world_gripper` | Required tool pose for this selected grasp |
| `receiver` | ID, side, seed, sample index, mesh source and `T_world_hand` |
| `receiver_protocol` | `fixed_world` policy, sampling bounds and planner settings |
| `hand_mesh_world` | Static hand vertices and triangle indices in world coordinates |
| `palm_position_world`, `palm_normal_world` | Reach-region origin and outward direction |
| `usage_boxes` | Supplied object-frame human-usage volumes |
| `planned_joints`, `executed_joints`, `dt_s` | Six-joint trajectories and sample interval |
| `planning` | Status, algorithm, seed, collision scope and measured planning time |

The object pose follows `T_world_gripper @ inverse(T_object_gripper)`.
The configured robot placement defines its world base frame. Calibration and
saved scene/path digests bind a replay to its input geometry.

Optional `object_boxes` and `hand_boxes_world` support auxiliary CPU geometry
checks. Each box defines `center`, positive `half_extents`, a proper `rotation`
and an optional `label`. Optional `delivery` supplies a method target and body
keypoints; `obstacle_boxes_world` specifies additional obstacles.

## Records and reporting

- **`run.json`** identifies the run, status and expected/completed trial count.
  The supervisor checks it after the simulator exits.
- **`*_trajectory.npz`** aligns timestamps, six arm joints, observed tool/object
  poses and contact flags; fixed receivers also export their transform and palm
  position. Read arrays with `numpy.load(path, allow_pickle=False)`.
- **`results.json/csv` and `report.html`** pair metric flags, first failure and
  success with scene and media links.
- **`paper_table.json/csv`** computes trial means per object, equally weighted
  object means per split, then equally weighted S0/S1 Avg. For Avg only, S0's
  Affordance contribution is zero, following the paper's footnote.
- **Timing** separates measured planning seconds, simulated execution seconds
  and simulator wall time. Total time combines planning and execution where
  both are present; unmeasured values use null.

A metric failure is a completed evaluated trial. Infrastructure errors use a
failed run status and nonzero CLI exit. `verify-output` checks artifact counts,
scene bindings, numeric arrays and result consistency. Video captions draw
outcomes from these same verified records.

## Paper-reference replay

`paper-replay` exports 8,000 `reconstructed_replay` records with reference
settings and outcomes matching Table I's 108 aggregate cells. Their source
settings, record population and aggregate checks accompany the export.
Simulator evaluation writes its own outcomes alongside the reference fields.

[Paper-to-code mapping](paper_details.md) · [Validation records](validation.md)
