from __future__ import annotations

from src.utils.geo import lla_to_enu, bearing_deg, wrap_angle


def test_wrap_angle():
    assert wrap_angle(190) == -170
    assert wrap_angle(-190) == 170


def test_bearing_equator_east():
    b = bearing_deg((0.0, 0.0), (0.0, 1.0))
    assert 85 <= b <= 95


def test_lla_to_enu_small_offset():
    home = (0.0, 0.0, 0.0)
    tgt = (0.0, 0.001, 0.0)  # ~111 m east at equator
    e, n, u = lla_to_enu(home, tgt)
    assert 100.0 < e < 120.0
    assert abs(n) < 10.0
    assert u == 0.0
