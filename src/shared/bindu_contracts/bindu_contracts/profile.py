from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class Profile:
    name: str
    groups: Dict[str, List[str]]
    limits: Dict[str, List[float]]
    max_speed: float
    capabilities: Tuple[str, ...]
    digest: str

    @classmethod
    def load(cls, path):
        raw = json.loads(Path(path).read_text())
        names = [j for group in raw['groups'].values() for j in group]
        if len(names) != len(set(names)) or not names:
            raise ValueError('joint names must be unique and nonempty')
        for name in names:
            lo, hi = raw['limits'][name]
            if not all(math.isfinite(x) for x in (lo, hi)) or not lo <= 0 <= hi or lo >= hi:
                raise ValueError('simulation limits must contain zero')
        if not math.isfinite(raw['max_speed']) or raw['max_speed'] <= 0:
            raise ValueError('invalid maximum speed')
        digest = hashlib.sha256(json.dumps(raw, sort_keys=True).encode()).hexdigest()
        return cls(raw['name'], raw['groups'], raw['limits'], raw['max_speed'],
                   tuple(raw['capabilities']), digest)
