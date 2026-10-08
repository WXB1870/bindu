"""Configured receiving-bin geometry shared by offline checks and Isaac views.

All positions are in the fixed robot base frame. The bin is an open box, not a
solid obstacle across its mouth. This module has no robot or simulator imports.
"""
import numpy as np


def pan_corners(recipe):
    center = np.asarray(recipe['pan_center_m'])
    half = np.asarray(recipe['pan_size_m']) / 2
    return np.array([np.r_[center+half*np.array([x,y,z]),1.]
                     for x in (-1,1) for y in (-1,1) for z in (-1,1)])


def bin_boxes(recipe):
    """Return (name, full size, world center) for four walls and the floor."""
    if 'receiving_bin' not in recipe:
        return []
    box = recipe['receiving_bin']
    center = np.asarray(box['center_xy_m'], dtype=float)
    size = np.asarray(box['inner_size_xy_m'], dtype=float)
    h, w, z = (float(box[n]) for n in ('height_m','wall_thickness_m','bottom_z_m'))
    if (center.shape != (2,) or size.shape != (2,) or
            not np.isfinite(np.r_[center,size,h,w,z]).all() or min(*size,h,w) <= 0 or w >= h):
        raise ValueError('INVALID_RECEIVING_BIN')
    x,y = center
    a,b = size
    return [('bottom',[a+2*w,b+2*w,w],[x,y,z+w/2]),
            ('front',[w,b+2*w,h],[x+(a+w)/2,y,z+h/2]),
            ('rear',[w,b+2*w,h],[x-(a+w)/2,y,z+h/2]),
            ('left',[a,w,h],[x,y+(b+w)/2,z+h/2]),
            ('right',[a,w,h],[x,y-(b+w)/2,z+h/2])]


def check_dump(samples, recipe):
    """Require a downward-facing pan over the open mouth during the dump hold.

    Projection containment is a geometric zero-horizontal-velocity envelope,
    not a snow friction, ballistic flight or mass-transfer prediction.
    """
    if 'dump' not in recipe:
        return None
    bin_boxes(recipe)  # validate dimensions before checking projections
    box, gate = recipe['receiving_bin'], recipe['dump']
    start,end = gate['hold_seconds']
    transfer_start, transfer_end = gate['transfer_seconds']
    if not 0 <= transfer_start <= start < end <= transfer_end:
        raise ValueError('INVALID_DUMP_WINDOW')
    margin, clearance = gate['mouth_margin_m'], gate['min_rim_clearance_m']
    tilt = gate['min_pan_tilt_deg']
    if not np.isfinite([start,end,transfer_start,transfer_end,margin,clearance,tilt]).all() or not (margin > 0 and clearance > 0 and 90 < tilt < 180):
        raise ValueError('INVALID_DUMP_ACCEPTANCE')
    center, half = np.asarray(box['center_xy_m']), np.asarray(box['inner_size_xy_m'])/2
    corner = pan_corners(recipe)
    in_transfer = [s for s in samples if transfer_start <= s['t'] <= transfer_end]
    hold = [s for s in samples if start <= s['t'] <= end]
    if (len(hold)<2 or len(in_transfer)<2 or hold[0]['t']>start+1e-6 or hold[-1]['t']<end-1e-6 or
            in_transfer[0]['t']>transfer_start+1e-6 or in_transfer[-1]['t']<transfer_end-1e-6):
        raise ValueError('MISSING_DUMP_SAMPLES')
    poses = np.asarray([s['tool'] for s in in_transfer])
    held = np.asarray([s['tool'] for s in hold])
    if not np.isfinite(poses).all() or not np.isfinite(held).all():
        raise ValueError('NONFINITE_DUMP_POSE')
    points = np.einsum('nij,kj->nki',poses,corner)
    mouth_margin = float(np.min(half - np.abs(points[:,:,:2]-center)))
    rim_clearance = float(points[:,:,2].min()-(box['bottom_z_m']+box['height_m']))
    min_tilt = float(np.rad2deg(np.arccos(np.clip(held[:,2,2],-1.,1.))).min())
    if mouth_margin < margin:
        raise ValueError('DUMP_MISSES_BIN')
    if rim_clearance < clearance:
        raise ValueError('DUMP_RIM_CLEARANCE')
    if min_tilt < tilt:
        raise ValueError('DUMP_PAN_NOT_INVERTED')
    return {'hold_samples':len(hold),'transfer_samples':len(in_transfer),
            'min_pan_tilt_deg':min_tilt,'min_mouth_margin_m':mouth_margin,
            'min_rim_clearance_m':rim_clearance,
            'scope':'pan envelope above bin; no snow or contact dynamics'}
