"""Inverse kinematics for ARES-1's four independently steerable, independently
driven (4WS4WD "swivel drive") wheel modules.

Given a desired chassis-frame twist (vx, vy, wz), this module computes, for
each of the four wheels, the steering angle (about the vertical axis) and
the wheel angular velocity (about the wheel's spin axis) required to realize
that twist without slip, assuming rigid, non-deformable rolling contact.

Wheel layout (base_link frame, +x forward, +y left):

    FL = (+wheelbase/2, +track_width/2)
    FR = (+wheelbase/2, -track_width/2)
    RL = (-wheelbase/2, +track_width/2)
    RR = (-wheelbase/2, -track_width/2)

For a rigid body with twist (vx, vy, wz), the velocity of the contact point
of wheel i located at (x_i, y_i) in the base frame is:

    Vx_i = vx - wz * y_i
    Vy_i = vy + wz * x_i

    wheel_speed_i        = sqrt(Vx_i^2 + Vy_i^2)
    steering_angle_i      = atan2(Vy_i, Vx_i)
    wheel_angular_vel_i   = wheel_speed_i / wheel_radius   (sign per below)

Steering-flip avoidance
------------------------
A naive atan2 solution can command a wheel to swivel by up to 180 degrees
between two nearly-identical motion requests whenever the required heading
crosses the +/-180 degree boundary, or whenever the requested speed is very
small and dominated by noise. Physically, a wheel pointed at angle theta and
spinning at speed s produces an identical contact-point velocity to a wheel
pointed at (theta + pi) spinning at speed -s. SwivelKinematics exploits this
symmetry: for each wheel it keeps the previous commanded steering angle and,
if the "flipped" solution (theta - pi, -speed) requires less rotation of the
steering joint from the previous command than the raw solution, it uses the
flipped solution instead. Below a configurable speed epsilon, the previous
steering angle is held rather than snapping to an arbitrary heading.
"""

from dataclasses import dataclass
import math
from typing import Dict, Tuple


WHEEL_NAMES = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class RoverGeometry:
    """Physical geometry required for the swivel inverse kinematics.

    All units are meters. These MUST match the values in
    ares1_description/urdf/ares1_properties.xacro.
    """
    wheelbase: float = 0.55
    track_width: float = 0.45
    wheel_radius: float = 0.11

    def wheel_positions(self) -> Dict[str, Tuple[float, float]]:
        """Return {wheel_name: (x, y)} in the base_link frame."""
        hb = self.wheelbase / 2.0
        ht = self.track_width / 2.0
        return {
            "FL": (hb, ht),
            "FR": (hb, -ht),
            "RL": (-hb, ht),
            "RR": (-hb, -ht),
        }


@dataclass(frozen=True)
class WheelCommand:
    """Result of the inverse kinematics for a single wheel."""
    steering_angle: float      # radians, wheel-frame yaw about the vertical axis
    wheel_angular_velocity: float  # rad/s, positive = forward roll


def normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def shortest_angular_distance(a_from: float, a_to: float) -> float:
    """Signed shortest angular distance from a_from to a_to, in (-pi, pi]."""
    return normalize_angle(a_to - a_from)


class SwivelKinematics:
    """Stateful inverse kinematics solver.

    Keeping per-wheel state (the previously commanded steering angle) lets
    the solver avoid unnecessary large steering excursions when the
    commanded twist is near zero or crosses an angle discontinuity.
    """

    def __init__(self, geometry: RoverGeometry = RoverGeometry(),
                 zero_speed_epsilon: float = 1.0e-4,
                 max_steering_angle: float = math.pi):
        self.geometry = geometry
        self.zero_speed_epsilon = zero_speed_epsilon
        self.max_steering_angle = max_steering_angle
        self._positions = geometry.wheel_positions()
        # Previous steering angle per wheel; starts pointed straight ahead.
        self._prev_angle: Dict[str, float] = {name: 0.0 for name in WHEEL_NAMES}

    def reset(self) -> None:
        """Reset all remembered steering angles to straight-ahead (0 rad)."""
        self._prev_angle = {name: 0.0 for name in WHEEL_NAMES}

    def compute(self, vx: float, vy: float, wz: float) -> Dict[str, WheelCommand]:
        """Compute steering angle and wheel angular velocity for all wheels.

        Args:
            vx: desired chassis-frame forward velocity (m/s), +x is forward.
            vy: desired chassis-frame lateral velocity (m/s), +y is left.
            wz: desired chassis-frame yaw rate (rad/s), + is counter-clockwise.

        Returns:
            {wheel_name: WheelCommand}
        """
        commands: Dict[str, WheelCommand] = {}
        r = self.geometry.wheel_radius

        for name in WHEEL_NAMES:
            x_i, y_i = self._positions[name]

            vx_i = vx - wz * y_i
            vy_i = vy + wz * x_i

            wheel_speed = math.hypot(vx_i, vy_i)

            if wheel_speed < self.zero_speed_epsilon:
                # No motion requested for this wheel: hold the previous
                # steering angle and command zero wheel velocity, rather
                # than snapping to an arbitrary/undefined atan2(0, 0) angle.
                steering_angle = self._prev_angle[name]
                angular_velocity = 0.0
            else:
                raw_angle = math.atan2(vy_i, vx_i)

                # Evaluate the "flipped" equivalent solution: pointing the
                # wheel 180 degrees the other way and reversing its spin
                # produces the same contact-point velocity.
                flipped_angle = normalize_angle(raw_angle + math.pi)

                dist_raw = abs(shortest_angular_distance(self._prev_angle[name], raw_angle))
                dist_flipped = abs(shortest_angular_distance(self._prev_angle[name], flipped_angle))

                if dist_flipped < dist_raw:
                    steering_angle = flipped_angle
                    speed_sign = -1.0
                else:
                    steering_angle = normalize_angle(raw_angle)
                    speed_sign = 1.0

                # Clamp to the physical steering travel limit. If the
                # target exceeds the limit, fall back to the un-flipped
                # solution (steering hardware has +/- max_steering_angle
                # of continuous travel in this design, so this only
                # triggers if max_steering_angle has been reduced below pi).
                if abs(steering_angle) > self.max_steering_angle:
                    steering_angle = normalize_angle(raw_angle)
                    speed_sign = 1.0

                angular_velocity = speed_sign * (wheel_speed / r)

            self._prev_angle[name] = steering_angle
            commands[name] = WheelCommand(
                steering_angle=steering_angle,
                wheel_angular_velocity=angular_velocity,
            )

        return commands


def compute_swivel_kinematics(
    vx: float,
    vy: float,
    wz: float,
    geometry: RoverGeometry = RoverGeometry(),
) -> Dict[str, WheelCommand]:
    """Stateless convenience wrapper (no flip-avoidance memory across calls).

    Useful for unit tests and one-shot computations where continuity between
    successive calls does not matter. For real-time control, prefer
    instantiating SwivelKinematics once and reusing it every control cycle
    so that steering-flip avoidance has state to work with.
    """
    solver = SwivelKinematics(geometry=geometry)
    return solver.compute(vx, vy, wz)
