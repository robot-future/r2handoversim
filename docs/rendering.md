# Laboratory rendering and camera views

`demo` now defaults to `--visual-style lab`: a neutral grey room and floor,
a dark workbench, soft area lighting, and matte hand/object materials. The
palette follows the benchmark paper's grey simulation scene and laboratory
photograph. Original UR5e/Robotiq materials and object vertex colors are retained.
The blue grid and its embedded point light are replaced in the render layer.

The lighting rig has a warm-neutral key, opposite-side fill, rear rim,
overhead softbox and receiver-side fill, plus low ambient illumination.
Five 1.6 × 1.2 m area lights illuminate different sides of the scene. This
reduces hard shadows and gives the hand and gripper readable surfaces from
several viewpoints. Surface normals smooth the rendered hand and object;
vertices, triangles, collision shapes and trajectories remain unchanged.

For final media, use path tracing (128 samples per frame, six bounces, spatial
OptiX denoising). Realtime RTX remains the default for faster benchmark runs.

```bash
python scripts/run_isaac.py demo --trial resolved_trial.json --headless \
  --visual-style lab --renderer pathtraced --camera handover \
  --video --screenshot --output outputs/lab_handover
```

Camera presets:

- `handover`: choose a receiver-side view that avoids arm occlusion.
- `left` / `right`: opposing oblique views, relative to the receiver-side view.
- `top`: elevated oblique view of the receiving region.
- `overview`: the fixed whole-scene view.

For original-asset fixed-hand scenes, the detail cameras check hand visibility
against robot/object bounds at the beginning, midpoint and end of the saved
trajectory, and consider object visibility at the final pose. The elevated
view searches the full azimuth range. They adjust only the camera. Different views can still have
occluded surfaces; use them together to inspect both fingers and the object.
Eye/target positions and visibility estimates are saved in each result.

```bash
for view in handover left right top; do
  python scripts/run_isaac.py demo --trial resolved_trial.json --headless \
    --visual-style lab --renderer pathtraced --camera "$view" \
    --video --video-speed 0.5 --screenshot --output "outputs/lab_$view"
done
```

Replay the same resolved trial for each camera so the numerical observations
can be compared. `verify-output` checks each export. Use `--visual-style debug`
to restore the original grid, lighting and visible Reach/skeleton guides.
Lab mode hides only those presentation guides; Reach and all collision metrics
continue to use the original geometry. The new background walls have no
colliders and cannot change planning outcomes.
