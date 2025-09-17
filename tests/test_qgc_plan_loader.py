from __future__ import annotations

import os
import pytest

from src.mission.qgc_plan_loader import load_qgc_plan


def test_load_qgc_plan_from_file():
    # Load the real QGC plan bundled with the tests
    plan_path = os.path.join(os.path.dirname(__file__), "test.plan")
    wps = load_qgc_plan(plan_path)

    # The plan contains 15 NAV_WAYPOINT (command 16) items and one LAND (ignored)
    assert len(wps) == 15

    # Validate first waypoint (from test.plan)
    assert wps[0].lat == pytest.approx(-33.5319069)
    assert wps[0].lon == pytest.approx(-70.76592362)
    assert wps[0].alt == pytest.approx(0.0)

    # Validate last waypoint (15th NAV_WAYPOINT before LAND)
    assert wps[-1].lat == pytest.approx(-33.53191652)
    assert wps[-1].lon == pytest.approx(-70.76576827)
    assert wps[-1].alt == pytest.approx(0.0)
