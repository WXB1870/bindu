"""Wire-compatible extraction of historical zmq_niic/bridge.py array framing.

Original: topic, JSON header, JSON payload, optional contiguous ndarray frames.
Only numeric arrays are accepted; all frames are bounded before restoration.
"""
from dataclasses import dataclass
import json
import math
import numpy as np

MAX_BYTES = 16 * 1024 * 1024


@dataclass(frozen=True)
class Message:
    topic: str
    seq: int
    timestamp_send: float
    payload: dict
    header: dict


def encode(topic, seq, timestamp, payload, extra=None):
    arrays = []

    def visit(value):
        if isinstance(value, np.ndarray):
            if value.dtype.kind not in 'buif':
                raise ValueError('PI_ARRAY_DTYPE')
            index = len(arrays)
            arrays.append(np.ascontiguousarray(value))
            return {'__ndarray__': index}
        if isinstance(value, dict):
            return {key: visit(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [visit(item) for item in value]
        return value

    body = visit(payload)
    header = dict(extra or {})
    header.update(topic=topic, seq=seq, timestamp_send=timestamp,
                  encoding='json+numpy.raw' if arrays else 'json',
                  arrays=[{'dtype': str(a.dtype), 'shape': list(a.shape)} for a in arrays])
    frames = [topic.encode(), json.dumps(header, ensure_ascii=False, allow_nan=False).encode(),
              json.dumps(body, ensure_ascii=False, allow_nan=False).encode()]
    frames.extend(a.tobytes() for a in arrays)
    if len(frames) > 16 or sum(map(len, frames)) > MAX_BYTES:
        raise ValueError('PI_MESSAGE_SIZE')
    return frames


def decode(frames):
    try:
        if not 3 <= len(frames) <= 16 or sum(map(len, frames)) > MAX_BYTES:
            raise ValueError('PI_MESSAGE_SIZE')
        topic = frames[0].decode()
        header, body = json.loads(frames[1]), json.loads(frames[2])
        if not isinstance(header, dict) or not isinstance(body, dict):
            raise ValueError('PI_MESSAGE_LAYOUT')
        if header.get('topic', topic) != topic:
            raise ValueError('PI_TOPIC_MISMATCH')
        seq, timestamp = header['seq'], header['timestamp_send']
        if type(seq) is not int or seq < 0 or not math.isfinite(timestamp):
            raise ValueError('PI_MESSAGE_ID')
        metas = header.get('arrays', [])
        if len(metas) != len(frames) - 3:
            raise ValueError('PI_ARRAY_COUNT')
        arrays = []
        for meta, raw in zip(metas, frames[3:]):
            dtype, shape = np.dtype(meta['dtype']), meta['shape']
            if (dtype.kind not in 'buif' or not isinstance(shape, list) or len(shape) > 4 or
                    any(type(n) is not int or n < 0 or n > MAX_BYTES for n in shape) or
                    math.prod(shape) * dtype.itemsize != len(raw)):
                raise ValueError('PI_ARRAY_LAYOUT')
            arrays.append(np.frombuffer(raw, dtype=dtype).reshape(shape))

        def restore(value):
            if isinstance(value, dict):
                if set(value) == {'__ndarray__'}:
                    index = value['__ndarray__']
                    if type(index) is not int or not 0 <= index < len(arrays):
                        raise ValueError('PI_ARRAY_INDEX')
                    return arrays[index]
                return {key: restore(item) for key, item in value.items()}
            if isinstance(value, list):
                return [restore(item) for item in value]
            return value

        return Message(topic, seq, float(timestamp), restore(body), header)
    except (KeyError, TypeError, OverflowError, RecursionError, UnicodeError) as exc:
        raise ValueError('PI_MESSAGE_LAYOUT') from exc
