"""Isaac Sim 5.0 standalone replay and PhysX hand-overlap evaluation.

Imports of Kit/USD happen only after SimulationApp starts. Scenes support bundled primitives and explicitly configured local USD/OBJ assets.
"""
import time
import json
from pathlib import Path
import numpy as np
from .evaluation import evaluate, robot_geometry, validate_trial, trial_tcp
from .geometry import box, box_pose, inverse, moved, transform, projected_width, points
from .robot import tcp
from .results import save_results


def replay(trials, output, headless=False, hold=False, render_every=1, screenshot=False, animation=False, run_id=None, hand_collision="boxes",
           video=False, video_speed=1., camera="overview", visual_style="lab", renderer="realtime"):
    if render_every < 1:
        raise ValueError("render_every must be at least 1")
    for trial in trials:
        validate_trial(trial)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    requested_hand_collision = hand_collision
    def save_run(status, error=None):
        from . import __version__
        (output / "run.json").write_text(json.dumps({"status": status, "run_id": run_id,
            "package_version": __version__, "camera": camera, "video": video,
            "visual_style": visual_style, "renderer": renderer,
            "requested_hand_collision": requested_hand_collision, "headless": headless,
            "expected_trials": len(trials), "completed_trials": len(results),
            "error": error}, indent=2))
    save_run("starting")
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        raise ImportError("Use Isaac Sim's python.sh, or an environment containing Isaac Sim 5.0. See README.") from exc
    # Do not pass our argparse flags to Kit's argument parser.
    import sys
    saved_argv = sys.argv
    sys.argv = [sys.argv[0]]
    app = SimulationApp({"headless": headless, "width": 1280, "height": 800,
                         "renderer": "PathTracing" if renderer=="pathtraced" else "RaytracedLighting",
                         "samples_per_pixel_per_frame":128,"max_bounces":6})
    sys.argv = saved_argv
    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils.viewports import set_camera_view
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysicsSchemaTools
        from omni.physx import get_physx_scene_query_interface, get_physx_interface
        import carb
        if renderer=="pathtraced":
            # Joint replay authors transforms directly; temporal denoising can leave stale silhouettes.
            carb.settings.get_settings().set_bool("/rtx/pathtracing/optixDenoiser/temporalMode/enabled",False)
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        stage = world.stage
        world.scene.add_default_ground_plane()
        dome = UsdLux.DomeLight.Define(stage, "/World/Light")
        dome.CreateIntensityAttr(1500.)
        set_camera_view(eye=np.array([1.3, -1.8, 1.8]), target=np.array([-.35, -.1, 1.05]))

        from .studio import setup as setup_studio, dress as dress_studio
        visual_record=setup_studio(stage) if visual_style=="lab" else {"preset":"debug"}

        def capture_file(path):
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            import asyncio
            path = Path(path).resolve()
            path.unlink(missing_ok=True)
            capture = capture_viewport_to_file(get_active_viewport(), str(path))
            done = asyncio.ensure_future(capture.wait_for_result())
            deadline = time.monotonic() + 60
            while not done.done():
                app.update()
                if time.monotonic() > deadline:
                    done.cancel()
                    raise RuntimeError("Viewport capture timed out after 60 seconds")
            done.result()
            from .video import complete_png
            while not complete_png(path):
                app.update()
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Viewport PNG write timed out: {path}")

        def quaternion(rotation):
            # USD/Gf uses row-vector matrices, NumPy kernel uses columns.
            matrix = Gf.Matrix3d(*np.asarray(rotation).T.reshape(-1).tolist())
            q = matrix.ExtractRotation().GetQuat()
            v = q.GetImaginary()
            return carb.Float4(float(v[0]), float(v[1]), float(v[2]), float(q.GetReal()))

        def draw_box(path, b, color, collider=False):
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(2.)
            cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.ClearXformOpOrder()
            t = box_pose(b)
            matrix = t.copy()
            matrix[:3, :3] = t[:3, :3] @ np.diag(b["half_extents"])
            op = xform.AddTransformOp()
            op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))
            if collider:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            return op

        def update_box(op, b):
            matrix = box_pose(b)
            matrix[:3, :3] = matrix[:3, :3] @ np.diag(b["half_extents"])
            op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))

        draw_box("/World/Table", box([-.35, 0, .70], [.65, .55, .035]), [.24, .29, .36], collider=True)
        save_run("running")
        requested_hand_collision = hand_collision
        for trial_index, trial in enumerate(trials):
            from copy import deepcopy
            trial = deepcopy(trial)
            hand_collision = trial.get('receiver_protocol',{}).get('hand_collision',requested_hand_collision)
            if trial_index:
                # A fresh USD stage also invalidates PhysX's cached geometry/actor handles.
                world.stop()
                get_physx_interface().release_physics_objects()
                World.clear_instance()
                from isaacsim.core.utils.stage import create_new_stage
                create_new_stage()
                app.update()
                world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60)
                stage=world.stage
                world.scene.add_default_ground_plane()
                dome=UsdLux.DomeLight.Define(stage,'/World/Light');dome.CreateIntensityAttr(1500.)
                visual_record=setup_studio(stage) if visual_style=='lab' else {'preset':'debug'}
                draw_box('/World/Table',box([-.35,0,.70],[.65,.55,.035]),[.24,.29,.36],collider=True)
                set_camera_view(eye=np.array([1.3,-1.8,1.8]),target=np.array([-.35,-.1,1.05]))
            world.stop()
            # PhysX actor handles can otherwise survive USD prim-path reuse across trials.
            get_physx_interface().release_physics_objects()
            app.update()
            if stage.GetPrimAtPath("/World/Trial"):
                stage.RemovePrim("/World/Trial")
            UsdGeom.Xform.Define(stage, "/World/Trial")
            for group in ("Robot", "Object", "Hand"):
                UsdGeom.Xform.Define(stage, f"/World/Trial/{group}")
            if stage.GetPrimAtPath('/World/AssetRobot'): stage.RemovePrim('/World/AssetRobot')
            asset_robot = None
            if 'asset_robot' in trial:
                from .usd_robot import UsdRobot
                config = trial['asset_robot']
                asset_robot = UsdRobot(stage, config['usd'], config['translation'])
                from .asset_preparation import prepare
                trial, opening = prepare(trial, asset_robot)
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Table')).MakeInvisible()
                UsdPhysics.CollisionAPI(stage.GetPrimAtPath('/World/Table')).GetCollisionEnabledAttr().Set(False)
            else:
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Table')).MakeVisible()
                UsdPhysics.CollisionAPI(stage.GetPrimAtPath('/World/Table')).GetCollisionEnabledAttr().Set(True)
            camera_record={"mode":camera}
            if camera != "overview":
                if ('hand_mesh_world' in trial and 'object_mesh_object' in trial
                        and trial.get('receiver_protocol',{}).get('policy')=='fixed_world'):
                    hand_points = np.asarray(trial['hand_mesh_world']['vertices'])
                    target_object = transform(trial['target_T_world_gripper']) @ inverse(trial['T_object_gripper'])
                    mesh_points = np.asarray(trial['object_mesh_object']['vertices'])
                    object_points = mesh_points@target_object[:3,:3].T+target_object[:3,3]
                    all_points = np.vstack([hand_points,object_points,transform(trial['target_T_world_gripper'])[:3,3]])
                    target = (all_points.min(0)+all_points.max(0))/2
                    scale = max(1.,np.linalg.norm(np.ptp(all_points,axis=0))/.45)
                    set_camera_view(eye=target+scale*np.array([.6,-.9,.6]),target=target)
                else:
                    target = .8*trial_tcp(trial, trial["planned_joints"][-1])[:3,3] + .2*np.asarray(trial["palm_position_world"])
                    eye=target+np.array([.48, -.65, .38])
                    if camera in ('left','right','top'):
                        from .camera import choose
                        eye,_=choose([trial['palm_position_world']],target,[[]],view=camera)
                    camera_record.update(eye_world=eye.tolist(),target_world=target.tolist())
                    set_camera_view(eye=eye, target=target)
            frame_directory = output / f"{trial['id']}_frames"
            if video:
                # A fresh directory prevents stale frames entering a rerun.
                import shutil
                shutil.rmtree(frame_directory, ignore_errors=True)
                frame_directory.mkdir()
            if "delivery" in trial and "keypoints_world" in trial["delivery"]:
                from .robot import segment_box
                delivery = trial["delivery"]
                kp = delivery["keypoints_world"]
                shoulder = delivery["shoulder_midpoint"]
                segments = [(kp["left_shoulder"], kp["right_shoulder"]),
                            (shoulder, delivery["torso_center"]), (shoulder, kp["elbow"]),
                            (kp["elbow"], kp["wrist"])]
                for i, (a, b) in enumerate(segments):
                    draw_box(f"/World/Trial/ReceiverSkeleton/bone_{i}", segment_box(a, b, .009), [.7, .4, .9])
                p = np.asarray(delivery["T_world_object"])[:3, 3]
                draw_box("/World/Trial/ReceiverSkeleton/target_direction",
                         segment_box(p, p+.12*np.asarray(delivery["hand_direction_world"]), .004), [.95, .5, .12])
            for i, b in enumerate(trial.get("obstacle_boxes_world", [])):
                # The default table is already present in the shared stage.
                if b.get("label") != "table":
                    draw_box(f"/World/Trial/Obstacles/part_{i}", b, [.45, .4, .4], collider=True)
            for i, b in enumerate(trial["hand_boxes_world"]):
                draw_box(f"/World/Trial/Hand/part_{i}", b, [1., .64, .31], collider=hand_collision == "boxes")
                if "hand_mesh_world" in trial:
                    UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/Trial/Hand/part_{i}")).MakeInvisible()
            if "hand_mesh_world" in trial:
                data = trial["hand_mesh_world"]
                mesh = UsdGeom.Mesh.Define(stage, "/World/Trial/Hand/mesh")
                mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in data["vertices"]])
                mesh.CreateFaceVertexCountsAttr([3] * len(data["faces"]))
                mesh.CreateFaceVertexIndicesAttr(np.asarray(data["faces"]).reshape(-1).tolist())
                mesh.CreateSubdivisionSchemeAttr("none")
                mesh.CreateDisplayColorAttr([Gf.Vec3f(.95, .70, .52)])
                if hand_collision == "mesh":
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
            world.set_simulation_dt(physics_dt=trial["dt_s"], rendering_dt=trial["dt_s"])
            q0 = trial["executed_joints"][0]
            robot_ops = [draw_box(f"/World/Trial/Robot/part_{i}", b, [.35, .6, .9])
                         for i, b in enumerate(robot_geometry(trial, q0))]
            if asset_robot:
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Trial/Robot')).MakeInvisible()
            grasp_inverse = inverse(trial["T_object_gripper"])
            object_ops = [draw_box(f"/World/Trial/Object/part_{i}", moved(b, trial_tcp(trial, q0) @ grasp_inverse), [.2, .85, .65])
                          for i, b in enumerate(trial["object_boxes"])]
            point_op = None
            if "object_mesh_object" in trial:
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Trial/Object')).MakeInvisible()
                data = trial['object_mesh_object']
                mesh = UsdGeom.Mesh.Define(stage, '/World/Trial/ObjectMesh')
                mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in data['vertices']])
                mesh.CreateFaceVertexCountsAttr([3]*len(data['faces']))
                mesh.CreateFaceVertexIndicesAttr(np.asarray(data['faces']).reshape(-1).tolist())
                mesh.CreateSubdivisionSchemeAttr('none')
                mesh.CreateDisplayColorAttr([Gf.Vec3f(*c) for c in data['colors']])
                mesh.GetDisplayColorPrimvar().SetInterpolation('vertex')
                point_op = UsdGeom.Xformable(mesh.GetPrim()).AddTransformOp()
                point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q0)@grasp_inverse).T.reshape(-1).tolist()))
                if trial.get('receiver_protocol',{}).get('object_collision') == 'convexHull':
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
            elif "object_points_object" in trial:
                for i in range(len(object_ops)):
                    UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/Trial/Object/part_{i}")).MakeInvisible()
                cloud = UsdGeom.Points.Define(stage, "/World/Trial/ObjectCloud")
                cloud.CreatePointsAttr([Gf.Vec3f(*p) for p in trial["object_points_object"]])
                cloud.CreateWidthsAttr([.0015]*len(trial["object_points_object"]))
                cloud.CreateDisplayColorAttr([Gf.Vec3f(.2, .85, .65)])
                point_op = UsdGeom.Xformable(cloud.GetPrim()).AddTransformOp()
                point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q0)@grasp_inverse).T.reshape(-1).tolist()))
            center = np.asarray(trial["palm_position_world"]) + trial["reach_offset_m"] * np.asarray(trial["palm_normal_world"])/np.linalg.norm(trial["palm_normal_world"])
            for axis in range(3):
                circle = UsdGeom.BasisCurves.Define(stage, f"/World/Trial/ReachRegion/ring_{axis}")
                circle.CreateTypeAttr("linear")
                circle.CreateWrapAttr("periodic")
                circle.CreateCurveVertexCountsAttr([64])
                coordinates = []
                for theta in np.linspace(0, 2*np.pi, 64, endpoint=False):
                    p = center.copy()
                    p[(axis+1)%3] += trial["reach_radius_m"]*np.cos(theta)
                    p[(axis+2)%3] += trial["reach_radius_m"]*np.sin(theta)
                    coordinates.append(Gf.Vec3f(*p.tolist()))
                circle.CreatePointsAttr(coordinates)
                circle.CreateWidthsAttr([.002])
                circle.SetWidthsInterpolation("constant")
                circle.CreateDisplayColorAttr([Gf.Vec3f(.15, .8, .3)])
            if visual_style=='lab': dress_studio(stage,asset_robot)
            get_physx_interface().force_load_physics_from_usd()
            world.reset()
            if asset_robot: asset_robot.update(q0, opening)
            world.step(render=True)
            if video or screenshot:
                for _ in range(8):
                    world.render()
            query = get_physx_scene_query_interface()
            if trial.get('receiver_protocol',{}).get('policy')=='fixed_world' and hand_collision=='mesh':
                hand_vertices=np.asarray(trial['hand_mesh_world']['vertices'])
                # Positive control: the live query pipeline must see the hand collider.
                seen=[]
                def hand_probe(hit):
                    if str(hit.collision)=='/World/Trial/Hand/mesh': seen.append(True)
                    return True
                center=(hand_vertices.min(0)+hand_vertices.max(0))/2
                extents=(hand_vertices.max(0)-hand_vertices.min(0))/2+.002
                # USD cooking can finish after reset, especially when switching object meshes.
                # Advance scene synchronization until the actual receiver shape is queryable.
                ready_started=time.monotonic()
                for ready_step in range(120):
                    query.overlap_box(carb.Float3(*extents),carb.Float3(*center),carb.Float4(0,0,0,1),hand_probe,False)
                    if seen: break
                    if time.monotonic()-ready_started>10.: break
                    world.step(render=False)
                    app.update()
                if not seen: raise RuntimeError('PhysX positive control did not detect the receiver mesh after scene synchronization')
                trial['receiver_protocol']['live_hand_collider_verified']=True
                trial['receiver_protocol']['hand_collider_ready_steps']=ready_step
                # Position-sensitive controls distinguish this hand from a stale nearby actor.
                surface_indices=np.linspace(0,len(hand_vertices)-1,8,dtype=int)
                for vertex in hand_vertices[surface_indices]:
                    seen.clear()
                    query.overlap_box(carb.Float3(.0015,.0015,.0015),carb.Float3(*vertex),carb.Float4(0,0,0,1),hand_probe,False)
                    if not seen: raise RuntimeError('PhysX receiver surface probe disagrees with the current hand mesh')
                trial['receiver_protocol']['hand_surface_probes_verified']=len(surface_indices)
                trial['receiver_protocol']['physics_scene_lifecycle']='fresh_usd_stage_per_trial'
            if trial.get('receiver_protocol',{}).get('replan_in_isaac'):
                if not asset_robot or hand_collision != 'mesh' or point_op is None:
                    raise ValueError('Fixed-receiver mesh planning requires original robot/object and receiving-hand mesh')
                from .physx_planning import plan as plan_mesh, forbidden, link_group
                def collision_clear(q):
                    if np.any(np.abs(q)>2*np.pi): return False,'joint_limits'
                    asset_robot.update(q,opening)
                    point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q)@grasp_inverse).T.reshape(-1).tolist()))
                    world.step(render=False)
                    for path in [*asset_robot.colliders,stage.GetPrimAtPath('/World/Trial/ObjectMesh').GetPath()]:
                        source_group=link_group(path); hits=[]
                        def callback(hit):
                            other_group=link_group(hit.collision)
                            if str(hit.collision)!=str(path) and forbidden(source_group,other_group):
                                hits.append(f'{source_group}:{other_group}')
                            return True
                        a,b=PhysicsSchemaTools.encodeSdfPath(path)
                        query.overlap_shape(a,b,callback,False)
                        if hits: return False,hits[0]
                    return True,None
                verts=np.asarray(trial['object_mesh_object']['vertices'])
                trial['stability_width_m']=float(np.ptp(verts@transform(trial['T_object_gripper'])[:3,1]))
                trial['stability_width_rule']='Full object mesh projection along closing axis (Eq. 3)'
                trial=plan_mesh(trial,collision_clear,iterations=trial['receiver_protocol'].get('iterations',200))
                trial['receiver_protocol']['replan_in_isaac']=False
                q0=trial['executed_joints'][0]
                asset_robot.update(q0,opening)
                point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q0)@grasp_inverse).T.reshape(-1).tolist()))
                world.step(render=True)
                print(f"{trial['id']}: plan {trial['planning']['status']}, {trial['planning']['checked_configurations']} checked states, {trial['planning']['rejected_contacts']}",flush=True)
            if (camera!='overview' and asset_robot and 'hand_mesh_world' in trial
                    and trial.get('receiver_protocol',{}).get('policy')=='fixed_world'):
                from .camera import choose
                # Preview the saved trajectory without changing any of its geometry.
                path=trial['executed_joints'];bounds_by_pose=[]
                meshes=[p for p in Usd.PrimRange(asset_robot.root) if p.GetTypeName()=='Mesh']
                for index in sorted({0,len(path)//2,len(path)-1}):
                    q=path[index];asset_robot.update(q,opening)
                    point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q)@grasp_inverse).T.reshape(-1).tolist()))
                    world.step(render=False)
                    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],False,True)
                    bounds=[]
                    for prim in [*meshes,stage.GetPrimAtPath('/World/Trial/ObjectMesh')]:
                        extent=cache.ComputeWorldBound(prim).ComputeAlignedRange()
                        if not extent.IsEmpty(): bounds.append([list(extent.GetMin()),list(extent.GetMax())])
                    bounds_by_pose.append(bounds)
                eye,diagnostic=choose(trial['hand_mesh_world']['vertices'],target,bounds_by_pose,scale,view=camera,object_vertices=object_points)
                camera_record.update(eye_world=eye.tolist(),target_world=target.tolist(),**diagnostic)
                set_camera_view(eye=eye,target=target)
                asset_robot.update(q0,opening)
                point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q0)@grasp_inverse).T.reshape(-1).tolist()))
                world.step(render=True)
                for _ in range(2): world.render()
            if asset_robot and trial['split']=='S1':
                # Query the real finger collider against the supplied semantic region volumes.
                # Regions are not physical obstacles; this cannot affect motion planning.
                from .mesh_metrics import affordance_digest
                object_pose=trial_tcp(trial,q0)@grasp_inverse
                usage_hits=[]
                def hit_usage(hit):
                    path=str(hit.collision)
                    if '/World/AssetRobot/robotiq_85_' in path and 'finger' in path:
                        usage_hits.append(path)
                    return True
                for region in trial['usage_boxes']:
                    region=moved(region,object_pose)
                    query.overlap_box(carb.Float3(*region['half_extents']),carb.Float3(*region['center']),
                                      quaternion(region['rotation']),hit_usage,False)
                trial['affordance_observation']={'clear':not usage_hits,'colliders':sorted(set(usage_hits)),
                    'geometry_sha256':affordance_digest(trial),
                    'source':'Isaac Sim original USD finger colliders versus supplied usage-region boxes'}
            contacts = []
            observed_tools = []
            started = time.perf_counter()
            for frame, q in enumerate(trial["executed_joints"]):
                if not app.is_running():
                    raise RuntimeError("Isaac Sim closed before replay finished")
                robot_boxes = robot_geometry(trial, q)
                if asset_robot: asset_robot.update(q, opening)
                for op, b in zip(robot_ops, robot_boxes):
                    update_box(op, b)
                for op, b in zip(object_ops, trial["object_boxes"]):
                    update_box(op, moved(b, trial_tcp(trial,q) @ grasp_inverse))
                if point_op is not None:
                    point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q)@grasp_inverse).T.reshape(-1).tolist()))
                world.step(render=video or frame % render_every == 0)
                if trial.get('receiver_protocol',{}).get('policy')=='fixed_world' and hand_collision=='mesh':
                    actual=np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Trial/Hand/mesh')).GetPointsAttr().Get())
                    if not np.allclose(actual,trial['hand_mesh_world']['vertices'],atol=1e-6,rtol=0):
                        raise RuntimeError('Receiver mesh moved during fixed-world execution')
                hit_hand = [False]

                def report_hit(hit):
                    if str(hit.collision).startswith("/World/Trial/Hand/"):
                        hit_hand[0] = True
                    return True

                if asset_robot:
                    for path in asset_robot.colliders:
                        a,b=PhysicsSchemaTools.encodeSdfPath(path)
                        query.overlap_shape(a,b,report_hit,False)
                else:
                    for b in robot_boxes:
                        query.overlap_box(carb.Float3(*b["half_extents"]), carb.Float3(*b["center"]),
                                          quaternion(b["rotation"]), report_hit, False)
                contacts.append(hit_hand[0])
                observed_tools.append(asset_robot.tool_pose()@transform(trial.get('T_asset_tool_grasp_frame',np.eye(4))) if asset_robot else trial_tcp(trial, q))
                if video:
                    capture_file(frame_directory / f"{frame:06d}.png")
            result = evaluate(trial, contacts)
            result["safe_source"] = f"Isaac Sim PhysX robot-box overlap against static hand {hand_collision} at every frame"
            if asset_robot:
                result['safe_source'] = f'Isaac Sim PhysX actual USD robot collider overlap against hand {hand_collision}'
                result['asset_robot'] = {**trial['asset_robot'], 'mesh_count':asset_robot.mesh_count,
                                        'collider_count':len(asset_robot.colliders), 'T_tcp_asset_tool':trial['T_tcp_asset_tool']}
                result['object_asset'] = {k:trial['object_mesh_object'].get(k) for k in ('source_path','source_sha256')}
                result['grasp_contact'] = trial['asset_contact_fit']
                result['asset_preparation'] = trial['asset_preparation']
            result["camera_configuration"] = camera_record
            result["visual_configuration"] = {**visual_record,"renderer":renderer,"pathtracing_samples":128 if renderer=="pathtraced" else None}
            result["hand_collision"] = hand_collision
            result['planning'] = trial.get('planning')
            result["execution_wall_time_s"] = time.perf_counter() - started
            result["backend"] = "isaacsim-physx"
            result["physics_scope"] = "Static hand colliders and " + ("USD robot collider" if asset_robot else "robot-box") + " overlap queries; kinematic robot and rigidly attached object; no grasp dynamics"
            results.append(result)
            # Some Kit installations terminate Python during app.close(). Persist first.
            save_results(results, output)
            save_run("running")
            # Self-contained USD includes procedural shapes and any supplied hand mesh.
            world.stop()
            snapshot = (output / f"{trial['id']}.usda").resolve()
            if not stage.Export(str(snapshot)):
                raise RuntimeError("USD scene export failed")
            from .trajectory_export import export as export_trajectory
            exports = export_trajectory(trial, contacts, output, observed_tools if asset_robot else None)
            result["artifacts"] = {"scene": snapshot.name, **exports}
            if video:
                from .video import encode_frames
                destination = output / f"{trial['id']}.mp4"
                encode_frames(frame_directory, destination, len(contacts), trial["dt_s"], video_speed)
                result["artifacts"]["video"] = destination.name
                result["recording"] = {"source": "Isaac Sim viewport", "camera": camera,
                    "captured_frames": len(contacts), "simulation_dt_s": trial["dt_s"],
                    "playback_speed": video_speed, "output_fps": 30, "start_hold_s": 1, "end_hold_s": 2}
                shutil.rmtree(frame_directory)
            if screenshot:
                for _ in range(4): world.render()
                capture_file(output / f"{trial['id']}.png")
                result["artifacts"]["screenshot"] = f"{trial['id']}.png"
            if animation:
                from .animation import bake
                bake(snapshot, output / f"{trial['id']}_animation.usda", trial, asset_robot=asset_robot, opening=opening if asset_robot else None)
                result["artifacts"]["animation"] = f"{trial['id']}_animation.usda"
            save_results(results, output)
            save_run("running")
            print(f"{trial['id']}: {result['first_failure'] or 'success'} (Isaac Sim)", flush=True)
        save_results(results, output)
        save_run("succeeded")
        while hold and not headless and app.is_running():
            app.update()
        return results
    except BaseException as exc:
        save_run("failed", str(exc))
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()
