"""
Trot gait planner for the Mini Pupper quadruped.

Trot gait overview
------------------
Diagonal leg pairs move in unison:
  Pair A (in-phase):    LF + RH   phase_offset = 0.0
  Pair B (anti-phase):  RF + LH   phase_offset = 0.5

The global phase φ ∈ [0, 1) increments at rate 1/T per second, where T is
the gait period. Each leg's local phase is:

    leg_phase = (φ + phase_offset[leg]) % 1.0

Stance vs swing
---------------
  leg_phase < duty_factor   → STANCE  (foot on ground, pushes backward)
  leg_phase ≥ duty_factor   → SWING   (foot in air, swings forward)

Foot trajectory
---------------
All positions are expressed in the hip joint frame of each leg:
  +x  forward, +y  lateral outward, +z  upward

Stance (linear push-back):
  t_s  = leg_phase / duty_factor           ∈ [0, 1)
  x(t) = +step_x/2 · (1 − 2·t_s)         sweeps forward→back
  y(t) = y_neutral + step_y/2 · (1 − 2·t_s)
  z(t) = z_nominal

Swing (sinusoidal arch):
  t_sw = (leg_phase − duty_factor) / (1 − duty_factor)   ∈ [0, 1)
  x(t) = −step_x/2 · (1 − 2·t_sw)        sweeps back→forward
  y(t) = y_neutral − step_y/2 · (1 − 2·t_sw)
  z(t) = z_nominal + step_height · sin(π · t_sw)

Step length per leg (body-frame velocity + yaw):
  step_x = (vx − ω · hip_y) · T
  step_y = (vy + ω · hip_x) · T

where (hip_x, hip_y) is the hip joint's xy position in the base_link frame.
"""

import numpy as np
from hyper_pupper_gait.kinematics import (
    HIP_POSITIONS, SIDE_SIGN, L_AB, LEG_NAMES
)


# Phase offsets for trot gait
PHASE_OFFSETS = {
    'lf': 0.0,
    'rh': 0.0,
    'rf': 0.5,
    'lh': 0.5,
}


class TrotGaitPlanner:
    """
    Generates per-leg foot positions (in hip frames) for a trot gait.

    Parameters
    ----------
    gait_period   : float   Full stride duration [s]. Default 0.5 s.
    duty_factor   : float   Fraction of cycle spent in stance. Default 0.5.
    step_height   : float   Peak foot lift during swing [m]. Default 0.03 m.
    body_height   : float   Nominal base_link height above ground [m]. Default 0.07 m.
    max_linear_vel: float   Velocity clamp for vx/vy [m/s]. Default 0.3 m/s.
    max_angular_vel: float  Velocity clamp for ω [rad/s]. Default 1.0 rad/s.
    """

    def __init__(
        self,
        gait_period=0.5,
        duty_factor=0.5,
        step_height=0.03,
        body_height=0.07,
        max_linear_vel=0.3,
        max_angular_vel=1.0,
    ):
        self.T = gait_period
        self.duty = duty_factor
        self.step_height = step_height
        self.body_height = body_height
        self.max_lin = max_linear_vel
        self.max_ang = max_angular_vel

        self._phase = 0.0   # global phase ∈ [0, 1)

        # Precompute nominal foot z (same for all legs because hip_z is equal)
        # and lateral neutral y in hip frame
        self._z_nominal = {}
        self._y_neutral = {}
        for leg in LEG_NAMES:
            hip_z = HIP_POSITIONS[leg][2]
            self._z_nominal[leg] = -(self.body_height + hip_z)
            self._y_neutral[leg] = SIDE_SIGN[leg] * L_AB

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self):
        """Reset global phase to zero."""
        self._phase = 0.0

    def step(self, dt, vx=0.0, vy=0.0, omega=0.0):
        """
        Advance the gait by dt seconds and return foot positions.

        Parameters
        ----------
        dt    : float   Time step [s].
        vx    : float   Desired forward velocity [m/s].
        vy    : float   Desired lateral velocity [m/s] (positive = left).
        omega : float   Desired yaw rate [rad/s] (positive = CCW from above).

        Returns
        -------
        foot_positions : dict { leg -> np.ndarray shape (3,) }
            Desired foot position in each leg's hip joint frame.
        is_stance : dict { leg -> bool }
            True if the leg is currently in stance phase.
        """
        # Clamp velocities
        vx    = np.clip(vx,    -self.max_lin, self.max_lin)
        vy    = np.clip(vy,    -self.max_lin, self.max_lin)
        omega = np.clip(omega, -self.max_ang, self.max_ang)

        # Advance global phase
        self._phase = (self._phase + dt / self.T) % 1.0

        foot_positions = {}
        is_stance = {}

        for leg in LEG_NAMES:
            leg_phase = (self._phase + PHASE_OFFSETS[leg]) % 1.0

            # Per-leg step extents, accounting for body rotation
            hip_x = HIP_POSITIONS[leg][0]
            hip_y = HIP_POSITIONS[leg][1]
            step_x = (vx - omega * hip_y) * self.T
            step_y = (vy + omega * hip_x) * self.T

            z_nom   = self._z_nominal[leg]
            y_neut  = self._y_neutral[leg]

            if leg_phase < self.duty:
                # ---- STANCE ----
                is_stance[leg] = True
                t_s = leg_phase / self.duty          # ∈ [0, 1)
                foot_x = step_x * (0.5 - t_s)
                foot_y = y_neut + step_y * (0.5 - t_s)
                foot_z = z_nom
            else:
                # ---- SWING ----
                is_stance[leg] = False
                t_sw = (leg_phase - self.duty) / (1.0 - self.duty)   # ∈ [0, 1)
                foot_x = step_x * (t_sw - 0.5)
                foot_y = y_neut + step_y * (t_sw - 0.5)
                foot_z = z_nom + self.step_height * np.sin(np.pi * t_sw)

            foot_positions[leg] = np.array([foot_x, foot_y, foot_z])

        return foot_positions, is_stance

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def phase(self):
        """Current global gait phase ∈ [0, 1)."""
        return self._phase

    @property
    def gait_period(self):
        return self.T

    @gait_period.setter
    def gait_period(self, value):
        self.T = max(0.1, float(value))
