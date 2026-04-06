"""Unit tests for obstacle avoidance logic."""

import math
import pytest


def compute_safe_velocity(
    linear, angular, front_dist, obstacle_angle,
    min_dist=0.35, slow_dist=0.8, escape_vel=0.8,
    max_linear=0.5, max_angular=1.5,
):
    """Pure-function version of obstacle avoidance logic for testing."""
    if front_dist < min_dist and linear >= 0.0:
        safe_linear = 0.0
        if obstacle_angle >= 0:
            safe_angular = -escape_vel
        else:
            safe_angular = escape_vel
    elif front_dist < slow_dist and linear > 0.0:
        scale = (front_dist - min_dist) / (slow_dist - min_dist)
        scale = max(0.0, min(1.0, scale))
        safe_linear = linear * scale
        safe_angular = angular
    else:
        safe_linear = linear
        safe_angular = angular

    safe_linear = max(-max_linear, min(max_linear, safe_linear))
    safe_angular = max(-max_angular, min(max_angular, safe_angular))
    return safe_linear, safe_angular


class TestObstacleAvoidance:

    def test_clear_path_passthrough(self):
        lin, ang = compute_safe_velocity(0.5, 0.0, 2.0, 0.0)
        assert lin == pytest.approx(0.5)
        assert ang == pytest.approx(0.0)

    def test_emergency_stop(self):
        lin, ang = compute_safe_velocity(0.5, 0.0, 0.2, 0.1)
        assert lin == pytest.approx(0.0)
        assert ang < 0  # rotate away from left obstacle

    def test_emergency_stop_right(self):
        lin, ang = compute_safe_velocity(0.5, 0.0, 0.2, -0.1)
        assert lin == pytest.approx(0.0)
        assert ang > 0  # rotate away from right obstacle

    def test_slowdown_zone(self):
        lin, ang = compute_safe_velocity(0.5, 0.0, 0.575, 0.0)
        # 0.575 is halfway between 0.35 and 0.8
        assert 0.0 < lin < 0.5
        assert ang == pytest.approx(0.0)

    def test_slowdown_proportional(self):
        lin1, _ = compute_safe_velocity(0.5, 0.0, 0.5, 0.0)
        lin2, _ = compute_safe_velocity(0.5, 0.0, 0.7, 0.0)
        assert lin1 < lin2  # closer = slower

    def test_reverse_not_affected(self):
        """Reversing should not be slowed by front obstacles."""
        lin, ang = compute_safe_velocity(-0.3, 0.0, 0.5, 0.0)
        assert lin == pytest.approx(-0.3)

    def test_reverse_not_blocked_in_emergency_zone(self):
        """Reverse should remain available to escape a front obstacle."""
        lin, ang = compute_safe_velocity(-0.3, 0.0, 0.2, 0.1)
        assert lin == pytest.approx(-0.3)
        assert ang == pytest.approx(0.0)

    def test_velocity_clamping(self):
        lin, ang = compute_safe_velocity(2.0, 5.0, 2.0, 0.0)
        assert lin == pytest.approx(0.5)
        assert ang == pytest.approx(1.5)
