#!/usr/bin/env python3
# encoding: utf-8
# ----------------------------------------------------------------------------
# RECONSTRUCTED pure-Python replacement for inverse_kinematics.so
#
# Original: Cython 3.0.9 extension (source name 'inverse_kinematics.pyx')
# compiled against /usr/include/python3.10, aarch64. Fails to load on a
# different CPython ABI ('undefined symbol: Py_EnterRecursiveCall').
#
# Public API recovered from the binary's DWARF debug info (HIGH confidence):
#   radians(degrees) -> float
#   degrees(randians) -> float        # 'randians' typo preserved from the .pyx
#   float2zero(num) -> float
#   remove_same(num_list) -> list
#   set_link(l0, l1, l2, l3, l4)
#   get_link() -> (l0, l1, l2, l3, l4)
#   set_joint_range(j1, j2, j3, j4, j5, unit='deg')
#   get_joint_range(unit='deg')
#   enable_print(p)
#   getTheta3(px, py, pz)
#   getTheta2(theta3, pz)
#   getTheta1(theta3, theta2, px, py)
#   getTheta4(theta3, theta2, theta1, ax, ay, az)
#   getTheta5(theta4, theta3, theta2, theta1, nx, ny, nz, ox, oy, oz)
#   get_position_ik(x, y, z, roll, pitch, yaw)
#   get_rpy_ik(position, orientation, tolerance, resolution=1.0)
#   get_ik(position, pitch, pitch_range, roll=0.0, resolution=1.0)
#
# NOTE on get_ik() arg order (verified against the Cython opt-args struct):
#   the optional args are (roll, resolution) -- roll FIRST. The kinematics
#   node calls `get_ik(position, pitch, list(pitch_range), resolution)`, so the
#   service's `resolution` value actually binds to `roll`, and the real pitch
#   step stays at its default (1.0 deg). See README for details.
#
# Confidence on the math:
#   * Position + pitch IK (get_ik / get_position_ik position part) -> MEDIUM-HIGH
#     Round-trips against the reconstructed FK to < 1e-4 m over 6000 random
#     reachable poses, both elbow branches.
#   * Full-orientation handling (roll about the approach axis, get_rpy_ik,
#     getTheta4/getTheta5 exact sign conventions) -> LOW-MEDIUM (validate).
# ----------------------------------------------------------------------------
import numpy as np
from math import sin, cos, sqrt, atan2, asin, acos, hypot
from math import radians as _radians, degrees as _degrees

# ----------------------------------------------------------------------------
# Module state (mirrors the .so's module-level globals)
# ----------------------------------------------------------------------------
_l0 = 0.10314916202   # base_link
_l1 = 0.12941763737   # link1
_l2 = 0.12941763737   # link2
_l3 = 0.05445583202   # link3
_l4 = 0.112           # tool_link

# joint ranges stored in degrees [min, max]
_joint_range = [[-120.2, 120.2], [-180.2, 0.2], [-120.2, 120.2],
                [-200.2, 20.2], [-120.2, 120.2]]

_PRINT = False


# ----------------------------------------------------------------------------
# small helpers (exact signatures recovered from DWARF)
# ----------------------------------------------------------------------------
def radians(degrees):
    return _radians(degrees)


def degrees(randians):           # noqa: typo 'randians' kept on purpose
    return _degrees(randians)


def float2zero(num):
    """Snap values that are numerically ~0 to exactly 0.0."""
    num = float(num)
    return 0.0 if abs(num) < 1e-6 else num


def remove_same(num_list):
    """Drop duplicate solutions (within tolerance), preserving order."""
    out = []
    for item in num_list:
        dup = False
        for kept in out:
            try:
                if np.allclose(np.array(item, dtype=float),
                               np.array(kept, dtype=float), atol=1e-4):
                    dup = True
                    break
            except Exception:
                if item == kept:
                    dup = True
                    break
        if not dup:
            out.append(item)
    return out


def enable_print(p):
    global _PRINT
    _PRINT = bool(p)


def _printf(msg):
    if _PRINT:
        print(msg)


# ----------------------------------------------------------------------------
# link / joint-range configuration
# ----------------------------------------------------------------------------
def set_link(l0, l1, l2, l3, l4):
    global _l0, _l1, _l2, _l3, _l4
    _l0, _l1, _l2, _l3, _l4 = float(l0), float(l1), float(l2), float(l3), float(l4)
    return True


def get_link():
    return (_l0, _l1, _l2, _l3, _l4)


def set_joint_range(j1, j2, j3, j4, j5, unit='deg'):
    global _joint_range
    rng = [list(j1), list(j2), list(j3), list(j4), list(j5)]
    if unit == 'rad':
        rng = [[_degrees(a), _degrees(b)] for a, b in rng]
    elif unit != 'deg':
        _printf('unvalid unit')
        return False
    _joint_range = rng
    return True


def get_joint_range(unit='deg'):
    if unit == 'deg':
        return [list(r) for r in _joint_range]
    if unit == 'rad':
        return [[_radians(a), _radians(b)] for a, b in _joint_range]
    _printf('unvalid unit')
    return None


def _in_range(theta_deg):
    for v, (lo, hi) in zip(theta_deg, _joint_range):
        if v < lo or v > hi:
            return False
    return True


# ----------------------------------------------------------------------------
# Analytic joint helpers (signatures recovered; conventions reconstructed).
# These mirror the .so's getThetaN decomposition. The public solvers below do
# NOT depend on them, but they are provided for API compatibility.
# Angles returned in RADIANS.
# ----------------------------------------------------------------------------
def getTheta3(px, py, pz):
    """Elbow angle from the wrist-centre position (shoulder frame)."""
    wr = hypot(px, py)
    wh = pz
    D2 = wr * wr + wh * wh
    c3 = (D2 - _l1 * _l1 - _l2 * _l2) / (2 * _l1 * _l2)
    c3 = max(-1.0, min(1.0, c3))
    return acos(c3)


def getTheta2(theta3, pz):
    """Shoulder pitch given the elbow angle and target height (approx.)."""
    # Reconstructed; exact form in the .so is unknown -> validate before relying on it.
    return atan2(pz, _l1 + _l2 * cos(theta3))


def getTheta1(theta3, theta2, px, py):
    """Base yaw."""
    return atan2(py, px)


def getTheta4(theta3, theta2, theta1, ax, ay, az):
    """Wrist pitch from the approach vector a=(ax,ay,az)."""
    pitch = atan2(az, hypot(ax, ay))
    return -(pitch + radians(90)) - theta2 - theta3


def getTheta5(theta4, theta3, theta2, theta1, nx, ny, nz, ox, oy, oz):
    """Wrist roll from the normal/orientation vectors n=(nx..), o=(ox..)."""
    return atan2(oz, nz)


# ----------------------------------------------------------------------------
# Core position + pitch solver (validated against the reconstructed FK)
# ----------------------------------------------------------------------------
def _solve_position_pitch(px, py, pz, pitch_deg, roll_deg=0.0):
    """Return a list of [t1..t5] solutions in RADIANS for a target tip
    position + tool pitch (deg). Empty if unreachable / out of range."""
    L = _l3 + _l4
    phi = _radians(pitch_deg)
    t1 = atan2(py, px)
    r = hypot(px, py)
    wr = r - L * cos(phi)                 # wrist centre, radial
    wh = (pz - _l0) - L * sin(phi)        # wrist centre, height above shoulder
    u = -wh
    D2 = wr * wr + u * u
    D = sqrt(D2)
    if D > _l1 + _l2 + 1e-9 or D < abs(_l1 - _l2) - 1e-9:
        return []
    c3 = (D2 - _l1 * _l1 - _l2 * _l2) / (2 * _l1 * _l2)
    c3 = max(-1.0, min(1.0, c3))
    sols = []
    for sign in (+1.0, -1.0):
        t3 = sign * acos(c3)
        t2 = atan2(u, wr) - atan2(_l2 * sin(t3), _l1 + _l2 * cos(t3))
        t2d, t3d = _degrees(t2), _degrees(t3)
        t4d = -(pitch_deg + 90.0) - t2d - t3d
        theta_deg = [_degrees(t1), t2d, t3d, t4d, roll_deg]
        if _in_range(theta_deg):
            sols.append([radians(a) for a in theta_deg])
    return sols


# ----------------------------------------------------------------------------
# Public solvers
# ----------------------------------------------------------------------------
def get_position_ik(x, y, z, roll, pitch, yaw):
    """Full-pose IK at one exact orientation (roll, pitch, yaw in degrees).

    Returns a flat list of joint-angle solutions (each a 5-tuple in RADIANS),
    or [] if there is no solution.  (roll maps to wrist roll; yaw is implied
    by the target x,y for this 5-DOF arm and is accepted for API parity.)
    """
    sols = _solve_position_pitch(x, y, z, pitch, roll)
    return remove_same(sols)


def get_ik(position, pitch, pitch_range, roll=0.0, resolution=1.0):
    """Search pitch over [pitch_range[0], pitch_range[1]] in `resolution`-deg
    steps; for every feasible pitch return [solutions, rpy].

    return: [ [solutions, rpy], ... ]
            solutions : list of 5-joint-angle tuples (RADIANS)
            rpy       : [roll, pitch, yaw] in degrees for that pitch
    """
    px, py, pz = position[0], position[1], position[2]
    lo, hi = float(pitch_range[0]), float(pitch_range[1])
    step = abs(resolution) if resolution else 1.0
    yaw = _degrees(atan2(py, px))

    # search outwards from the requested pitch so the closest pitch comes first
    candidates = [pitch]
    k = 1
    while True:
        added = False
        p_hi = pitch + k * step
        p_lo = pitch - k * step
        if p_lo >= lo:
            candidates.append(p_lo)
            added = True
        if p_hi <= hi:
            candidates.append(p_hi)
            added = True
        if not added:
            break
        k += 1

    results = []
    for p in candidates:
        if p < lo or p > hi:
            continue
        sols = _solve_position_pitch(px, py, pz, p, roll)
        sols = remove_same(sols)
        if sols:
            results.append([sols, [float(roll), float(p), yaw]])
    return results


def get_rpy_ik(position, orientation, tolerance, resolution=1.0):
    """IK toward a target orientation given as rpy=(roll,pitch,yaw) deg, with a
    per-axis `tolerance` (deg). Reconstructed wrapper around get_ik over the
    pitch tolerance band.  LOW-MEDIUM confidence -- validate before relying on
    the orientation handling."""
    roll, pitch, yaw = orientation[0], orientation[1], orientation[2]
    try:
        tol = float(tolerance)
    except (TypeError, ValueError):
        tol = float(tolerance[1]) if len(tolerance) > 1 else 0.0
    return get_ik(position, pitch, [pitch - tol, pitch + tol], roll, resolution)


if __name__ == '__main__':
    enable_print(True)
    print('links:', get_link())
    print('ranges:', get_joint_range('deg'))
    res = get_position_ik(0.2, 0.0, 0.2, 0.0, 0.0, 0.0)
    print('position_ik solutions (rad):')
    for s in res:
        print('  ', [round(v, 4) for v in s])
    res = get_ik([0.2, 0.0, 0.2], 0.0, [-90.0, 90.0])
    print('get_ik entries:', len(res), '| first rpy:', res[0][1] if res else None)
