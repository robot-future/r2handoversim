# Fixed-world random receiver replay

The asset workflow renders the actual UR5e/Robotiq USD, original object mesh and
an explicitly configured left or right hand mesh. Each sampled hand stays fixed
through candidate selection, planning and recording. Start with the
[step-by-step setup](quickstart.md) to build your first original-asset scene.

## Inputs and ordering

To generate the same type of visible neutral hand templates locally, use the
standalone exporter (no sibling repo or Torch dependency):

```bash
# In a separate Python environment with NumPy, SciPy, six and legacy Chumpy:
python -m pip install 'numpy<2' scipy six setuptools
python -m pip install --no-build-isolation --no-deps chumpy==0.70
python scripts/export_mano_receivers.py --models /path/to/MANO/models \
  --output outputs/local_hands
```

Only supply your trusted, licensed `MANO_LEFT.pkl` and `MANO_RIGHT.pkl` files.
The resulting `outputs/local_hands/config.json` includes palm frames, mesh units
and explicit sampling bounds. The exporter uses neutral rest templates. Alternatively, provide existing OBJ meshes
and their correctly measured palm frames.


Install `pip install -e '.[assets,planning]'` in the Isaac Sim Python environment.
Supply the local robot/object paths in `configs/local_assets.example.json`.
Supply licensed hand OBJ files and their palm frame in
`configs/receivers.example.json`. Paths are relative to that config. Positions
and mesh scale must be explicit; there is no automatic unit guessing. The palm
frame uses +Z outward and +X toward the fingers. The mesh can be a MANO export;
the repository does not redistribute MANO model weights or hand templates.

The example places the palm in world X `[-0.72, -0.60]`, Y `[-0.45, -0.30]`,
Z `[0.88, 0.98]` metres. With the supported robot base at `[0, 0, 0.75]`,
this gives a horizontal base-to-palm distance of 0.67–0.85 m. Reference IK filters
unreachable proposals before method selection. These configurable demo bounds
are recorded with each sample. Existing saved trials retain their original hand poses.

1. Calibrate **all** candidate grasps against the original robot pads before
   the method filters/ranks candidates:

   ```bash
   python scripts/run_isaac.py prepare-candidates --scene input_scene.json \
     --asset-config local_assets.json --output calibrated_scene.json
   ```

   Candidate IDs, order and source preparation are preserved, including failed
   fits. Each valid candidate includes its contact width, both pad contacts,
   mesh digest, robot configuration and tool-frame calibration. The method must
   recompute its filters and scores on these prepared poses. Replaying an
   unprepared method selection is rejected; it is never silently refitted.

2. Sample the receiver **before selecting a grasp**:

   ```bash
   r2handoversim receiver-scenes --scene calibrated_scene.json \
     --receiver-config receivers.json --samples 4 --seed 27 \
     --output outputs/receiver_scenes
   ```

   Each scene contains the sampled hand in object coordinates and its fixed
   world pose. Methods must select independently on every scene in `scenes.json`.
   All modes for the same scene share its hand and object target. Object pose
   uses the first valid prepared candidate as an orientation reference, with
   object bounding center 15 cm along the outward palm normal. This is an
   authored offline target with explicit geometry and provenance.

3. For a method run using the benchmark full-object width definition, set
   `gripper.feasibility_width_policy` to `object_projection` in each scene before
   selection, or run the companion method's
   `ablate --scene sampled_scene.json --feasibility-width-policy object_projection`.
   Use the scene saved by that command for conversion.
   This filters all candidates by the same 85 mm predicate while preserving
   local pad aperture for physical geometry. Keep this as a separate named
   configuration from the default local-aperture method runs.

   Run the companion method on each scene, then convert its selection:

   ```bash
   r2handoversim from-intent --scene sampled_scene.json \
     --selection selected.json --output trial.json
   python scripts/run_isaac.py demo --trial trial.json --headless \
     --camera handover --video --screenshot --output outputs/record
   r2handoversim verify-output --input outputs/record
   r2handoversim review-video --input outputs/record --output review.mp4
   ```

   Fixed-world scene conversion defers planning to Isaac Sim. The command uses
   original USD colliders, the attached object convex hull and static hand
   triangles. It never moves the receiver to accommodate a grasp. Each trial
   starts a fresh USD/PhysX world. Eight position-sensitive surface probes
   verify the current hand collider, and its vertices are checked during
   execution. Failure records remain at home and are retained.
   A failure clip is therefore a stationary diagnostic scene, not a successful
   delivery animation.

For a complete companion neural run, `from-pipeline --pipeline pipeline.json`
retains the neural scene/selected grasp and supplied ergonomic delivery target.
If the scene carries calibrated asset candidates and a decoded hand mesh,
planning is deferred to the same original-collider Isaac backend. The input
pipeline path/hash is retained for traceability. This route uses the predicted
hand and supplied body keypoints; it is separate from the random template bank.

`sample-receivers` is also available for standalone asset geometry trials. Do
not use it to replace the hand after a method has selected a grasp and then
claim that method was evaluated on the new hand.

## Sampling and recorded evidence

Uniform XYZ and roll/pitch/yaw bounds are explicit config values. Seeds use an
object-specific deterministic stream; the left/right templates cycle evenly.
The provided config and hand exporter enable `require_reference_ik: true`,
which conditions proposals on full-pose IK for a preselection reference grasp,
using the calibrated `T_tcp_asset_tool` to define the reachable set. Every accepted/rejected proposal is saved in the receiver
bank. Explicitly setting it to false gives bounded-workspace geometry demos,
including the earlier local integration samples, without the reachability
condition. The sampler records which definition was used. Collision or method
failures preserve the sampled receiver.

The JSON trial preserves `receiver.id`, side, seed, sample index, mesh source
hash, `T_world_hand`, target object pose, method selection and grasp contract.
The NPZ records each frame's arm joints, observed USD tool pose, rigid object
pose, contact flag, fixed hand transform and palm position. `verify-output`
checks scene/trajectory/metric agreement. Video captions use those same results. The handover camera evaluates original
USD world bounds at three trajectory poses to reduce hand occlusion; camera
coordinates and the visibility estimate are saved separately from metrics.

The planner is numerical full-pose IK plus RRT-Connect using live PhysX scene
queries. Its explicit allowed-contact policy excludes internal
Robotiq contacts, links within two arm hops, mounted shoulder/table contact and
held-object/gripper/distal-wrist contact. Edges and every executed state are
checked; trajectory sampling is 0.6 rad/s at 60 Hz by default. Execution uses
kinematic robot motion and rigid object attachment.

## Metrics

- Stability: full original mesh projection onto the grasp closing axis, at
  most 85 mm. This is separate from the locally fitted pad aperture; both
  definitions are explicitly recorded in the grasp contract.
  Failing Stability skips IK/search and leaves execution at home.
- Plan: full tool pose plus collision-checked path; original robot USD, object
  hull, static hand mesh, table and supplied obstacles. The saved scene/path
  digest rejects changes to a previously validated plan.
- Reach: actual object convex hull intersects the sphere centered at palm +
  0.12 m normal, radius 0.10 m.
- Affordance: S1 only; live original finger colliders queried against supplied
  usage-region boxes. The region itself remains an annotation approximation
  unless the user supplies an authoritative volume.
- Safe: actual USD arm/gripper colliders versus hand triangles at every frame.
- First failure: Stability → Plan → Reach → Affordance → Safe.

The local setup uses two neutral MANO templates with random SE(3) placements,
16 original object meshes and 6,827 source candidates. Receiver templates,
candidate sets and semantic regions are configurable scene inputs. Four offline
baseline settings and aggregate reconstructed replay records have separate
provenance from the evaluated simulator runs.
