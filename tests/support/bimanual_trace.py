"""Validate measured tool traces without importing a runnable scenario."""
import json

import numpy as np


def check_tool_trace(rows, inputs, config, description, output, captures=None):
    """Align measured FK with the latest published request, then check all CAD poses."""
    import pinocchio as pin
    from bisect import bisect_right
    from bindu_kinematics.tool_geometry import collision_check, tracking_acceptance, coupling_metrics
    if not rows or not inputs:
        raise ValueError('EMPTY_TOOL_TRACKING_TRACE')
    stamps=[p['stamp'] for p in inputs]
    if not all(np.isfinite(stamps)) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('INVALID_TOOL_INPUT_TIMESTAMPS')
    model=pin.buildModelFromUrdf(str(config['urdf']));data=model.createData()
    frame=model.getFrameId(config['left_frame']);mount=np.array(config['left_to_tool']).reshape(4,4)
    right_frame=model.getFrameId(config['right_frame'])
    grip_offsets={side:np.r_[config['demo'].get(side+'_grip_offset_m',
        config['demo']['grip_offset_m']),1.] for side in ('left','right')}
    target_grips={side:np.linalg.inv(np.array(config[side+'_to_tool']).reshape(4,4)) @ offset
                  for side,offset in grip_offsets.items()}
    required=set(model.names[1:])
    dense=[];errors=[];previous=float('-inf')
    for row in rows:
        stamp=row['stamp']
        if stamp==previous:continue
        if not np.isfinite(stamp) or stamp<previous:raise ValueError('INVALID_TOOL_FEEDBACK_TIME')
        previous=stamp
        if set(row['positions']) != required:raise ValueError('TOOL_FEEDBACK_JOINT_LAYOUT')
        q=pin.neutral(model)
        for name,value in row['positions'].items():q[model.joints[model.getJointId(name)].idx_q]=value
        if not np.isfinite(q).all():raise ValueError('NONFINITE_TOOL_FEEDBACK')
        pin.framesForwardKinematics(model,data,q)
        tool=data.oMf[frame].homogeneous @ mount
        dense.append({'t':stamp,'q':row['positions'],'tool':tool.tolist(),
            'flanges':{config['left_frame']:data.oMf[frame].homogeneous.tolist(),
                       config['right_frame']:data.oMf[right_frame].homogeneous.tolist()}})
        index=bisect_right(stamps,stamp)-1
        if index<0:continue
        target=np.array(inputs[index]['pose']).reshape(4,4)
        if not np.isfinite(target).all():raise ValueError('NONFINITE_TOOL_REQUEST')
        grip_errors={side+'_grip_position_m':float(np.linalg.norm((
            data.oMf[frame_id].homogeneous @ grip_offsets[side]-target @ target_grips[side])[:3]))
            for side,frame_id in (('left',frame),('right',right_frame))}
        errors.append({'stamp':stamp,'input_stamp':stamps[index],**grip_errors,
                       'position_m':float(np.linalg.norm(tool[:3,3]-target[:3,3])),
                       'rotation_rad':float(np.linalg.norm(pin.log3(tool[:3,:3].T @ target[:3,:3])))})
    if not errors:raise ValueError('EMPTY_TOOL_TRACKING_MATCHES')
    result={'samples':len(errors),'alignment':'latest input stamp <= feedback stamp; no time shift',
            'input_max_gap_s':max((b-a for a,b in zip(stamps,stamps[1:])),default=0.),
            'max_position_m':max(e['position_m'] for e in errors),
            'max_rotation_rad':max(e['rotation_rad'] for e in errors)}
    result['endpoints']={side:dict(max_position_m=max(e[side+'_grip_position_m'] for e in errors),
        rms_position_m=float(np.sqrt(np.mean([e[side+'_grip_position_m']**2 for e in errors]))),
        final_position_m=errors[-1][side+'_grip_position_m']) for side in ('left','right')}
    positions=np.asarray([s['tool'] for s in dense])[:,:3,3]
    result['motion_extent_m']=float(np.max(np.linalg.norm(positions-positions[0],axis=1)))
    result['max_lift_m']=float(positions[:,2].max()-positions[0,2])
    (output/'tracking_trace.json').write_text(json.dumps(errors)+'\n')
    (output/'measured_motion.json').write_text(json.dumps(dense)+'\n')
    result['acceptance']=tracking_acceptance(config['demo'],result['max_position_m'],result['max_rotation_rad'])
    if captures is not None:
        result['coupling']=coupling_metrics(dense,config,captures)
        (output/'coupling_validation.json').write_text(json.dumps(result['coupling'],indent=2)+'\n')
    recipe=dict(config['demo'])
    recipe['left_grip_in_tool_m']=(np.linalg.inv(mount) @ np.r_[recipe['grip_offset_m'],1.])[:3].tolist()
    result['geometry']=collision_check(description/recipe['display_urdf'],dense,recipe)
    (output/'trace_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    if captures is not None and not result['coupling']['passed']:
        raise ValueError('BIMANUAL_COUPLING_ERROR:'+json.dumps(result['coupling']))
    if not result['acceptance']['passed']:
        raise ValueError('TOOL_TRACKING_ERROR:'+json.dumps(result))
    return result
