from __future__ import annotations

import json
import tempfile

from src.mission.qgc_plan_loader import load_qgc_plan


def test_load_qgc_plan_basic():
    plan = {
        "mission": {
            "items": [
                {
                    "command": 16,
                    "coordinate": [10.0, 20.0, 30.0],
                    "param1": 2.0,
                    "param2": 1.5,
                },
                {"command": 22},  # ignored
                {
                    "command": 16,
                    "params": [0, 0, 0, 0, 11.0, 21.0, 31.0],
                },
            ]
        }
    }
    with tempfile.NamedTemporaryFile("w+", suffix=".plan") as f:
        json.dump(plan, f)
        f.flush()
        wps = load_qgc_plan(f.name)
    assert len(wps) == 2
    assert wps[0].lat == 10.0 and wps[0].lon == 20.0 and wps[0].alt == 30.0
    assert wps[1].lat == 11.0 and wps[1].lon == 21.0 and wps[1].alt == 31.0
