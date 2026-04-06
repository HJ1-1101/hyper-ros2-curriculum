"""
Analytical inverse kinematics for the Mini Pupper quadruped robot.

Robot geometry (from hyper_pupper_complete/urdf/properties.urdf.xacro):

  base_link
  └── {leg}_hip_joint   (revolute, X-axis, abduction/adduction)
      └── {leg}_hip_link
          └── {leg}_upper_leg_joint  (revolute, Y-axis, at y=±L_AB from hip)
              └── {leg}_upper_leg_link
                  └── {leg}_lower_leg_joint  (revolute, Y-axis, at z=-L1 from upper)
                      └── {leg}_lower_leg_link
                          └── {leg}_foot_joint  (fixed, at z=-L2 from lower)

Leg parameters:
  L_AB = 0.0197 m   hip abduction offset (y-axis, + for left legs, - for right)
  L1   = 0.0500 m   thigh length (upper_leg_joint to lower_leg_joint, z-component)
  L2   = 0.0560 m   shin length  (lower_leg_joint to foot)

Hip joint positions relative to base_link:
  LF: (+0.06014, +0.0235, +0.0171)
  LH: (-0.05886, +0.0235, +0.0171)
  RF: (+0.06014, -0.0235, +0.0171)
  RH: (-0.05886, -0.0235, +0.0171)

Coordinate convention (hip joint frame):
  +x  forward
  +y  lateral outward (left for left legs, right for right legs)
  +z  upward  (foot below hip → pz < 0 in standing)

IK sign convention:
  theta_hip   > 0  abducts outward  (away from body)
  theta_upper < 0  swings thigh forward (foot forward)
  theta_lower > 0  bends knee (typical standing configuration)
"""

import numpy as np


# ---------------------------------------------------------------------------
# Robot constants
# ---------------------------------------------------------------------------

L_AB = 0.0197   # m  abduction link length
L1   = 0.0500   # m  thigh length
L2   = 0.0560   # m  shin  length

# Hip joint positions in base_link frame [x, y, z]
HIP_POSITIONS = {
    'lf': np.array([ 0.06014,  0.0235, 0.0171]),
    'lh': np.array([-0.05886,  0.0235, 0.0171]),
    'rf': np.array([ 0.06014, -0.0235, 0.0171]),
    'rh': np.array([-0.05886, -0.0235, 0.0171]),
}

# +1 for left legs, -1 for right legs (sign of the abduction y-offset)
SIDE_SIGN = {'lf': 1, 'lh': 1, 'rf': -1, 'rh': -1}

LEG_NAMES = ['lf', 'lh', 'rf', 'rh']


# ---------------------------------------------------------------------------
# Single-leg IK
# ---------------------------------------------------------------------------

def leg_ik(foot_pos_hip, leg):
    """
    Compute joint angles for one leg given the desired foot position.

    Parameters
    ----------
    foot_pos_hip : array-like, shape (3,)
        Desired foot position (px, py, pz) expressed in the hip joint frame.
        Convention: px forward, py lateral-outward, pz up (negative when foot
        is below the hip, as in normal standing).

    leg : str
        One of 'lf', 'lh', 'rf', 'rh'.

    Returns
    -------
    (theta_hip, theta_upper, theta_lower) : tuple of float
        Joint angles in radians.
        Returns (0, 0, 0) and logs a warning if the foot is unreachable.

    Notes
    -----
    Derivation summary
    ~~~~~~~~~~~~~~~~~~
    The forward kinematics (hip frame → foot position) are:

        foot = R_x(θ_hip) · [ (0, s·L_AB, 0)
                               + R_y(θ_upper) · [ (0, 0, -L1)
                                                  + R_y(θ_lower) · (0, 0, -L2) ] ]

    where s = +1 (left) or -1 (right).

    Step 1 — Hip abduction (frontal plane yz):
        D²  = py² + pz²
        pzs = -√(D² − L_AB²)      (effective vertical after removing ab-offset)
        θ_hip = atan2(pz, py) − atan2(pzs, s·L_AB)

    Step 2 — 2-link sagittal IK (thigh + shin in xz-plane):
        d²    = px² + pzs²
        cos_k = (d² − L1² − L2²) / (2·L1·L2)
        θ_lower = arccos(clip(cos_k, −1, 1))
        φ     = atan2(px, −pzs)
        α     = atan2(L2·sin(θ_lower), L1 + L2·cos(θ_lower))
        θ_upper = φ − α
    """
    px, py, pz = float(foot_pos_hip[0]), float(foot_pos_hip[1]), float(foot_pos_hip[2])
    s = SIDE_SIGN[leg]

    # ---- Step 1: hip abduction ----
    D_sq = py**2 + pz**2
    ab_sq = L_AB**2
    if D_sq < ab_sq:
        # Foot is within the abduction circle — clamp to boundary
        D_sq = ab_sq + 1e-9

    pzs = -np.sqrt(D_sq - ab_sq)          # always negative (foot below plane)
    theta_hip = np.arctan2(pz, py) - np.arctan2(pzs, s * L_AB)

    # ---- Step 2: sagittal 2-link IK ----
    d_sq = px**2 + pzs**2
    d    = np.sqrt(d_sq)

    reach_max = L1 + L2
    reach_min = abs(L1 - L2)
    if d > reach_max:
        # Over-extended: stretch as far as possible
        d = reach_max - 1e-6
        d_sq = d**2
    elif d < reach_min:
        d = reach_min + 1e-6
        d_sq = d**2

    cos_knee = (d_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    theta_lower = np.arccos(np.clip(cos_knee, -1.0, 1.0))

    phi   = np.arctan2(px, -pzs)          # angle from −z toward +x
    alpha = np.arctan2(L2 * np.sin(theta_lower),
                       L1 + L2 * np.cos(theta_lower))
    theta_upper = phi - alpha

    return theta_hip, theta_upper, theta_lower


# ---------------------------------------------------------------------------
# Single-leg FK  (for testing / debugging)
# ---------------------------------------------------------------------------

def leg_fk(angles, leg):
    """
    Forward kinematics: joint angles → foot position in hip frame.

    Parameters
    ----------
    angles : array-like, shape (3,)
        (theta_hip, theta_upper, theta_lower)
    leg : str

    Returns
    -------
    foot_pos : np.ndarray, shape (3,)
        Foot position (px, py, pz) in the hip joint frame.
    """
    theta_hip, theta_upper, theta_lower = angles
    s = SIDE_SIGN[leg]

    def Rx(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])

    def Ry(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])

    # Shin tip in lower_leg frame
    p = np.array([0.0, 0.0, -L2])
    # Propagate through lower_leg joint
    p = np.array([0.0, 0.0, -L1]) + Ry(theta_lower) @ p
    # Propagate through upper_leg joint (add abduction offset origin)
    p = np.array([0.0, s * L_AB, 0.0]) + Ry(theta_upper) @ p
    # Propagate through hip joint
    p = Rx(theta_hip) @ p

    return p


# ---------------------------------------------------------------------------
# All-legs IK (convenience wrapper)
# ---------------------------------------------------------------------------

def all_legs_ik(foot_positions_hip):
    """
    Compute joint angles for all four legs.

    Parameters
    ----------
    foot_positions_hip : dict  {leg_name -> np.ndarray shape (3,)}
        Desired foot positions in each leg's hip frame.

    Returns
    -------
    joint_angles : dict  {leg_name -> (theta_hip, theta_upper, theta_lower)}
    """
    return {leg: leg_ik(foot_positions_hip[leg], leg) for leg in LEG_NAMES}


# ---------------------------------------------------------------------------
# Default (neutral standing) foot positions
# ---------------------------------------------------------------------------

def default_foot_positions(body_height=0.07):
    """
    Return the neutral foot positions (in each hip frame) for a flat-standing
    robot at the given body height above the ground plane.

    body_height : float
        Height of base_link above the ground [m]. Default 0.07 m.
    """
    positions = {}
    for leg in LEG_NAMES:
        hip_z = HIP_POSITIONS[leg][2]          # hip above base_link
        z_foot = -(body_height + hip_z)        # foot z in hip frame (negative)
        y_foot = SIDE_SIGN[leg] * L_AB         # lateral offset
        positions[leg] = np.array([0.0, y_foot, z_foot])
    return positions
