"""Explicit PICO kinematic fixture; original shovel recipes remain unchanged."""
from copy import deepcopy
import numpy as np
from .vr.mapping import pose_matrix


def short_rod_config(config):
    """Rebase the tool origin to the midpoint of the two calibrated grip points.

    Geometry, IK, mapping and display share this transform. Rod length follows
    grip spacing, not the original shovel length. Translation is half scale;
    rotation stays 1:1. This function is only selected by the simulation demo.
    """
    if config.get('simulation_only') is not True:
        raise ValueError('SHORT_ROD_REQUIRES_SIMULATION')
    cfg=deepcopy(config)
    recipe=cfg['demo']
    left=pose_matrix(cfg['left_to_tool']);right=pose_matrix(cfg['right_to_tool'])
    left_point=(np.linalg.inv(left) @ np.r_[recipe['grip_offset_m'],1.])[:3]
    right_point=(np.linalg.inv(right) @ np.r_[recipe['right_grip_offset_m'],1.])[:3]
    center=(left_point+right_point)/2
    spacing=float(np.linalg.norm(left_point-right_point))
    axis=np.asarray(recipe['shaft_axis_in_tool'],dtype=float)
    if not .05 <= spacing <= 1.2 or not np.allclose(left_point-right_point,axis*spacing,atol=1e-5):
        raise ValueError('SHORT_ROD_GRIP_GEOMETRY')
    offset=np.eye(4);offset[:3,3]=center
    cfg['left_to_tool']=(left @ offset).ravel().tolist()
    cfg['right_to_tool']=(right @ offset).ravel().tolist()
    cfg['position_scale']=.5
    cfg['two_hand_rod']=True
    cfg['rod_axis_in_tool']=axis.tolist()
    cfg['integration_max_sample_gap']=.25
    recipe.update(geometry_kind='rod',shaft_from_left_grip_m=[-spacing-.11,.11],
                  left_grip_in_tool_m=(left_point-center).tolist(),
                  description='Kinematic PICO demo: short rod centred between the two grip points.')
    recipe['acceptance']['ground_policy']='report_only'
    for key in ('receiving_bin','dump','keyframes','pivot_in_tool_m','selected_design','design_search','grasp_schedule'):
        recipe.pop(key,None)
    return cfg
