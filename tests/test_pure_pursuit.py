from __future__ import annotations

from src.core.controllers import pure_pursuit


def test_pure_pursuit_straight():
    heading, speed = pure_pursuit((0.0, 0.0), 0.0, [(10.0, 0.0)], 1.0)
    assert abs(heading - 0.0) < 1e-3
    assert speed > 0.5


def test_pure_pursuit_turn_left():
    heading, speed = pure_pursuit((0.0, 0.0), 0.0, [(10.0, 10.0)], 1.0)
    assert 30 <= heading <= 60
    assert 0.0 <= speed <= 1.0
