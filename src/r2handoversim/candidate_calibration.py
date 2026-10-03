"""Calibrate every candidate before method filtering/ranking, never after selection."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid
import numpy as np


def export(scene_path, config_path, output):
    output=Path(output).resolve();output.parent.mkdir(parents=True,exist_ok=True)
    run_id=str(uuid.uuid4())
    with tempfile.TemporaryDirectory(prefix='r2-candidates-') as tmp:
        job=Path(tmp)/'job.json';state=Path(tmp)/'status.json'
        job.write_text(json.dumps(dict(scene=str(Path(scene_path).resolve()), config=str(Path(config_path).resolve()),
            output=str(output),state=str(state),run_id=run_id)))
        env=os.environ.copy();source=str(Path(__file__).resolve().parent.parent)
        env['PYTHONPATH']=source+os.pathsep+env.get('PYTHONPATH','')
        completed=subprocess.run([sys.executable,'-m','r2handoversim.candidate_calibration',str(job)],env=env)
        status=json.loads(state.read_text()) if state.is_file() else {}
        if completed.returncode != 0 or status.get('status') != 'succeeded' or status.get('run_id') != run_id:
            raise RuntimeError(f"Candidate calibration incomplete: {status.get('error','worker did not complete')}")
        return status


def calibrate(scene, config, robot):
    from .local_assets import attach
    from .asset_preparation import mesh_digest
    from .grasp_fit import fit_grasp
    from .geometry import inverse, points, transform
    from .robot import HOME, tcp
    result=deepcopy(scene)
    if result.get('schema_version') != 'handover.scene.v1' or result.get('units') != 'm':
        raise ValueError('Expected handover.scene.v1 in meters')
    attached=attach([{'object_id':scene['object']['id']}],config)[0]
    mesh=attached['object_mesh_object'];robot_config=attached['asset_robot'];digest=mesh_digest(mesh)
    shift=np.eye(4);shift[2,3]=.010
    result['object']['mesh']=mesh
    result['gripper'].update(width_policy='asset_mesh_pad',grasp_frame='parallel_jaw_tip',
        score_reference_point_gripper=[0.,0.,-.016],
        geometry='Original Robotiq 2F-85 flat pad surfaces', geometry_contract={
            'schema_version':'handover.asset_gripper.v1','T_asset_tool_grasp_frame':shift.tolist(),
            'pad_window_gripper_m':{'x':[-.010,.010],'z':[-.032,0.]},
            'robot':robot_config,'mesh_sha256':digest})
    ids=set();passed=0
    for candidate in result['candidates']:
        if candidate['id'] in ids: raise ValueError('Duplicate candidate ID')
        ids.add(candidate['id'])
        original=transform(candidate['T_object_gripper'])
        prep={'schema_version':'handover.asset_candidate.v1','original_T_object_gripper':original.tolist(),
              'robot':deepcopy(robot_config),'mesh_sha256':digest,'T_asset_tool_grasp_frame':shift.tolist(),
              'input_geometry_preparation':deepcopy(candidate.get('geometry_preparation')),
              'candidate_id':candidate['id'],'scope':'Original USD contact fitting before method selection'}
        try:
            grasp,fit=fit_grasp(mesh,original@inverse(shift),result['gripper']['max_opening_m'])
            opening,pads=robot.calibrate_opening(HOME,fit['width_m'])
            distances=robot.contact_distances(fit['contact_points_tool'])
            if max(distances)>.0002 or not np.isfinite(distances).all():
                raise ValueError('Fitted object contacts do not reach both USD pads')
            method_grasp=grasp@shift
            prep.update(status='bilateral_surface_fit',T_object_gripper=method_grasp.tolist(),width_m=fit['width_m'],
                contact_points_asset_tool=fit['contact_points_tool'],
                contact_points_gripper=points(inverse(shift),fit['contact_points_tool']).tolist(),
                bilateral_distance_m=distances, linkage_command_m=opening,
                T_tcp_asset_tool=(inverse(tcp(HOME))@robot.tool_pose()@shift).tolist(),
                measured_pad_geometry=pads,pad_window_gripper_m={'x':[-.010,.010],'z':[-.032,0.]})
            candidate['T_object_gripper']=method_grasp.tolist()
            distance=float(np.linalg.norm(np.ptp(np.asarray(mesh['vertices']),axis=0))+.1)
            candidate['approach_ray_origin_object']=(method_grasp[:3,3]-distance*method_grasp[:3,2]).tolist()
            passed+=1
        except ValueError as exc:
            prep.update(status='failed',reason=str(exc))
        candidate['geometry_preparation']=prep
    result['asset_calibration']={'schema_version':'handover.asset_calibration.v1',
        'candidates':len(ids),'contact_valid':passed,'contact_failed':len(ids)-passed,
        'candidate_order_preserved':True,'selection_performed':False}
    return result


def worker(job):
    state=Path(job['state'])
    def save(status,**fields):
        state.write_text(json.dumps(dict(run_id=job['run_id'],status=status,**fields)))
    save('starting')
    app=None
    try:
        from isaacsim import SimulationApp
        sys.argv=[sys.argv[0]]
        app=SimulationApp({'headless':True})
        from pxr import Usd
        from .local_assets import configuration
        from .usd_robot import UsdRobot
        config=configuration(job['config'])
        robot=UsdRobot(Usd.Stage.CreateInMemory(),config['robot_usd'],config.get('robot_translation',[-.5625,-.36,-.02]))
        result=calibrate(json.loads(Path(job['scene']).read_text()),config,robot)
        result['asset_calibration']['run_id']=job['run_id']
        output=Path(job['output']);temp=output.with_suffix('.partial.json')
        temp.write_text(json.dumps(result,allow_nan=False));temp.replace(output)
        save('succeeded',candidates=result['asset_calibration']['candidates'],
             contact_valid=result['asset_calibration']['contact_valid'])
    except BaseException as exc:
        save('failed',error=str(exc));raise
    finally:
        if app: app.close()


if __name__=='__main__':
    worker(json.loads(Path(sys.argv[1]).read_text()))
