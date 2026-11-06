import math

import pytest

from ares1_control.swivel_kinematics import (
    RoverGeometry,
    SwivelKinematics,
    WHEEL_NAMES,
    compute_swivel_kinematics,
    normalize_angle,
    shortest_angular_distance,
)

GEOM = RoverGeometry(wheelbase=0.55, track_width=0.45, wheel_radius=0.11)


def test_wheel_positions_match_spec():
    pos = GEOM.wheel_positions()
    assert pos["FL"] == pytest.approx((0.275, 0.225))
    assert pos["FR"] == pytest.approx((0.275, -0.225))
    assert pos["RL"] == pytest.approx((-0.275, 0.225))
    assert pos["RR"] == pytest.approx((-0.275, -0.225))


def test_pure_forward_motion():
    cmds = compute_swivel_kinematics(1.0, 0.0, 0.0, geometry=GEOM)
    for name in WHEEL_NAMES:
        assert cmds[name].steering_angle == pytest.approx(0.0, abs=1e-9)
        assert cmds[name].wheel_angular_velocity == pytest.approx(1.0 / GEOM.wheel_radius)


def test_pure_reverse_motion():
    cmds = compute_swivel_kinematics(-1.0, 0.0, 0.0, geometry=GEOM)
    # With flip-avoidance starting from a fresh (angle=0) solver, the raw
    # solution atan2(0, -1) = pi is exactly as far from 0 as its flip (0),
    # so the flipped solution (angle stays 0, speed negated) is selected —
    # this is the physically sensible "drive backwards, wheels unturned".
    for name in WHEEL_NAMES:
        assert cmds[name].steering_angle == pytest.approx(0.0, abs=1e-9)
        assert cmds[name].wheel_angular_velocity == pytest.approx(-1.0 / GEOM.wheel_radius)


def test_pure_crab_lateral_motion():
    cmds = compute_swivel_kinematics(0.0, 1.0, 0.0, geometry=GEOM)
    for name in WHEEL_NAMES:
        # All wheels must point +90 degrees (toward +y) with equal speed.
        assert cmds[name].steering_angle == pytest.approx(math.pi / 2.0, abs=1e-9)
        assert cmds[name].wheel_angular_velocity == pytest.approx(1.0 / GEOM.wheel_radius)


def test_diagonal_motion():
    cmds = compute_swivel_kinematics(1.0, 1.0, 0.0, geometry=GEOM)
    expected_angle = math.atan2(1.0, 1.0)  # 45 degrees, all wheels identical (wz=0)
    expected_speed = math.hypot(1.0, 1.0) / GEOM.wheel_radius
    for name in WHEEL_NAMES:
        assert cmds[name].steering_angle == pytest.approx(expected_angle, abs=1e-9)
        assert cmds[name].wheel_angular_velocity == pytest.approx(expected_speed, abs=1e-9)


def test_in_place_rotation_symmetry():
    cmds = compute_swivel_kinematics(0.0, 0.0, 1.0, geometry=GEOM)
    pos = GEOM.wheel_positions()
    for name in WHEEL_NAMES:
        x_i, y_i = pos[name]
        vx_i = -1.0 * y_i
        vy_i = 1.0 * x_i
        expected_angle = math.atan2(vy_i, vx_i)
        expected_speed = math.hypot(vx_i, vy_i) / GEOM.wheel_radius
        # Either the raw or flipped-equivalent solution is acceptable, but
        # the resulting contact-point velocity must match.
        angle = cmds[name].steering_angle
        speed = cmds[name].wheel_angular_velocity
        vx_cmd = math.cos(angle) * speed * GEOM.wheel_radius
        vy_cmd = math.sin(angle) * speed * GEOM.wheel_radius
        assert vx_cmd == pytest.approx(vx_i, abs=1e-9)
        assert vy_cmd == pytest.approx(vy_i, abs=1e-9)
        assert expected_speed >= 0.0  # sanity


def test_general_motion_all_components_nonzero():
    cmds = compute_swivel_kinematics(0.5, 0.3, 0.4, geometry=GEOM)
    pos = GEOM.wheel_positions()
    for name in WHEEL_NAMES:
        x_i, y_i = pos[name]
        vx_i = 0.5 - 0.4 * y_i
        vy_i = 0.3 + 0.4 * x_i
        angle = cmds[name].steering_angle
        speed = cmds[name].wheel_angular_velocity
        vx_cmd = math.cos(angle) * speed * GEOM.wheel_radius
        vy_cmd = math.sin(angle) * speed * GEOM.wheel_radius
        assert vx_cmd == pytest.approx(vx_i, abs=1e-9)
        assert vy_cmd == pytest.approx(vy_i, abs=1e-9)


def test_zero_velocity_is_safe_and_holds_previous_angle():
    solver = SwivelKinematics(geometry=GEOM)
    first = solver.compute(0.0, 1.0, 0.0)  # crab -> all wheels at +90 deg
    second = solver.compute(0.0, 0.0, 0.0)  # stop
    for name in WHEEL_NAMES:
        assert second[name].wheel_angular_velocity == pytest.approx(0.0)
        # Steering angle should be held from before, not reset to 0.
        assert second[name].steering_angle == pytest.approx(first[name].steering_angle)


def test_flip_avoidance_reduces_large_steering_excursions():
    solver = SwivelKinematics(geometry=GEOM)
    # Start driving forward: all wheels at angle 0.
    solver.compute(1.0, 0.0, 0.0)
    # Command a tiny negative vx: raw atan2(0, -eps) ~ +/-pi (near the
    # discontinuity). Flip-avoidance should choose the ~0 rad solution
    # with reversed wheel speed instead of swinging ~180 degrees.
    cmds = solver.compute(-0.01, 0.0, 0.0)
    for name in WHEEL_NAMES:
        assert abs(cmds[name].steering_angle) < math.pi / 2.0
        assert cmds[name].wheel_angular_velocity < 0.0


def test_normalize_angle_range():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(2 * math.pi) == pytest.approx(0.0, abs=1e-9)
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi, abs=1e-9) or \
        normalize_angle(3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)
    assert -math.pi < normalize_angle(5.0) <= math.pi


def test_shortest_angular_distance():
    assert shortest_angular_distance(0.0, math.pi / 2.0) == pytest.approx(math.pi / 2.0)
    assert shortest_angular_distance(math.pi - 0.01, -math.pi + 0.01) == pytest.approx(0.02, abs=1e-6)


def test_steering_angle_never_exceeds_limit():
    solver = SwivelKinematics(geometry=GEOM, max_steering_angle=math.pi)
    for vx, vy, wz in [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0),
                        (1, 1, 1), (-1, -1, -1), (0, 0, 2), (0.01, 0.01, 0.01)]:
        cmds = solver.compute(vx, vy, wz)
        for name in WHEEL_NAMES:
            assert abs(cmds[name].steering_angle) <= math.pi + 1e-9


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
