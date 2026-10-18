"""Read-only input diagnostics; timestamps describe reception, not headset sampling."""
from collections import deque
import math


class VRConnectivity:
    def __init__(self, max_age=.5):
        self.max_age = max_age
        self.source = None
        self.frames = {}
        self.samples = deque(maxlen=240)

    def ingest(self, frames):
        if not isinstance(frames, tuple):
            frames = (frames,)
        if not frames:
            return
        first = frames[0]
        if first.source_id != self.source:
            self.source = first.source_id
            self.frames.clear()
            self.samples.clear()
        if self.samples and first.seq <= self.samples[-1][0]:
            return
        self.samples.append((first.seq, first.stamp))
        for frame in frames:
            self.frames[frame.side] = frame

    def snapshot(self, now):
        sides = {}
        for side in ('left', 'right'):
            frame = self.frames.get(side)
            age = now-frame.stamp if frame else None
            fresh = age is not None and 0 <= age <= self.max_age
            valid = bool(frame and frame.valid and fresh)
            sides[side] = dict(valid=valid, age_s=age, seq=frame.seq if frame else None,
                position_m=[frame.pose[i] for i in (3, 7, 11)] if valid else None,
                squeeze=frame.grip if valid else None,
                trigger=frame.trigger if valid else None,
                buttons=frame.buttons if valid else None,
                button_edges=list(frame.button_edges) if valid else None)
        if not self.samples:
            mode, reason = 'INPUT_WAITING', 'Enter VR, allow tracking, and move both controllers'
        elif not any(v['age_s'] is not None and 0 <= v['age_s'] <= self.max_age for v in sides.values()):
            mode, reason = 'INPUT_STALE', 'No fresh controller events; check headset/browser/network'
        elif all(v['valid'] for v in sides.values()):
            mode, reason = 'INPUT_LIVE', 'Both controller payloads are valid'
        elif any(v['valid'] for v in sides.values()):
            mode, reason = 'INPUT_PARTIAL', 'One controller is missing or invalid'
        else:
            mode, reason = 'INPUT_INVALID', 'Events received, but controller payloads are invalid'
        recent = [(seq, stamp) for seq, stamp in self.samples if 0 <= now-stamp <= 2.]
        dt = recent[-1][1]-recent[0][1] if len(recent) > 1 else 0.
        hz = (recent[-1][0]-recent[0][0])/dt if dt > 0 else None
        return dict(connectivity=True, mode=mode, reason=reason, sides=sides,
                    receive_hz=hz if hz is not None and math.isfinite(hz) else None,
                    source_id=self.source, target=None, measured=None, error=None)


def status_lines(view):
    lines = [('BINDU | VR CONNECTIVITY | NO ROBOT', '#f1f5f9'),
             (view['mode'], '#86efac' if view['mode'] == 'INPUT_LIVE' else '#fbbf24')]
    for side, labels in (('left', ('X', 'Y')), ('right', ('A', 'B'))):
        state = view['sides'][side]
        label = side[0].upper()
        if state['valid']:
            lines.append((label+' XYZ(m): %.3f / %.3f / %.3f' % tuple(state['position_m']), '#cbd5e1'))
            bits = state['buttons']
            lines.append((f"{label} S:{state['squeeze']:.2f} T:{state['trigger']:.2f} "
                          f"{labels[0]}:{int(bool(bits & 1))} {labels[1]}:{int(bool(bits & 2))} "
                          f"Stick:{int(bool(bits & 4))}", '#7dd3fc'))
        else:
            lines.extend([(label+' XYZ(m): --', '#cbd5e1'),
                          (label+' buttons: -- (missing / invalid / stale)', '#fbbf24')])
    hz = view['receive_hz']
    lines.append((f'PC receive: ~{hz:.1f} Hz | WebXR coordinates' if hz is not None
                  else 'PC receive: -- Hz | WebXR coordinates', '#cbd5e1'))
    return lines
