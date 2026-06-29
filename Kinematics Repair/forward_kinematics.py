#!/usr/bin/env python3
# encoding: utf-8
# ----------------------------------------------------------------------------
# RECONSTRUCTED pure-Python replacement for forward_kinematics.so
#
# The original was a Cython 3.0.9 extension compiled against /usr/include/
# python3.10 for aarch64 (Jetson). Loading it under any other CPython ABI
# (e.g. 3.8) fails with `undefined symbol: Py_EnterRecursiveCall`.
#
# This file reimplements the *observable* behaviour of that module in pure
# Python, so it works on any interpreter and removes the ABI dependency.
#
# Confidence:
#   * Public API (names / args / return shapes / units)  -> HIGH
#     (recovered from the binary's DWARF debug info + call sites in the node)
#   * Modified-DH forward-kinematics math                -> MEDIUM-HIGH
#     (derived from the DH table documented in transform.py; self-consistent
#      and exactly invertible by the reconstructed inverse_kinematics.py)
#   * Absolute frame/zero conventions vs the *physical* arm -> NEEDS VALIDATION
#     Validate with the known home pose: all servos at pulse 500
#     -> joint angles (0, -90, 0, -90, 0) deg -> tool straight up.
# ----------------------------------------------------------------------------
import numpy as np
from math import sin, cos, sqrt, atan2, hypot, radians, degrees

try:                                   # real ROS type when available ...
    from geometry_msgs.msg import Pose, Quaternion
    _HAVE_ROS = True
except Exception:                      # ... lightweight stand-in for testing
    _HAVE_ROS = False

    class Quaternion:                  # noqa: D401  (matches geometry_msgs API)
        __slots__ = ("x", "y", "z", "w")

        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

        def __repr__(self):
            return "Quaternion(x=%g, y=%g, z=%g, w=%g)" % (self.x, self.y, self.z, self.w)


def _rot2qua(M):
    """3x3 rotation matrix -> geometry_msgs/Quaternion (same method as transform.rot2qua)."""
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]) / 3.0
    vals, vecs = np.linalg.eigh(K)
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if q[0] < 0:
        q = -q
    out = Quaternion()
    out.w, out.x, out.y, out.z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return out


def _mdh(alpha, a, theta, d):
    """Modified (Craig) Denavit-Hartenberg homogeneous transform i-1 -> i."""
    ca, sa, ct, st = cos(alpha), sin(alpha), cos(theta), sin(theta)
    return np.array([
        [ct,      -st,     0.0,    a],
        [st * ca,  ct * ca, -sa,  -sa * d],
        [st * sa,  ct * sa,  ca,   ca * d],
        [0.0, 0.0, 0.0, 1.0]])


class ForwardKinematics:
    """5-DOF robotic arm forward kinematics (reconstructed).

    Original docstring recovered from the binary:
        '5dof Robotics Arm Forward Kinematic by aiden data:2023/03/20'
    """

    def __init__(self, debug=False):
        self.debug = bool(debug)
        # Default link lengths (m) -- taken from transform.py, i.e. THIS arm.
        # (The compiled .so shipped slightly different built-in defaults,
        #  evidence it was built for another arm revision; call set_link()
        #  to be explicit.)
        self.base_link = 0.10314916202
        self.link1 = 0.12941763737
        self.link2 = 0.12941763737
        self.link3 = 0.05445583202
        self.tool_link = 0.076
        # Joint limits in degrees, [min, max], matching transform.py.
        self.joint_range = [[-120.2, 120.2], [-180.2, 0.2], [-120.2, 120.2],
                            [-200.2, 20.2], [-120.2, 120.2]]

    # -- configuration -------------------------------------------------------
    def set_link(self, base_link, link1, link2, link3, tool_link):
        self.base_link = float(base_link)
        self.link1 = float(link1)
        self.link2 = float(link2)
        self.link3 = float(link3)
        self.tool_link = float(tool_link)
        return True

    def get_link(self):
        # (base_link, link1, link2, link3, end_effector_link)
        return (self.base_link, self.link1, self.link2, self.link3, self.tool_link)

    def set_joint_range(self, j1, j2, j3, j4, j5, unit='deg'):
        rng = [list(j1), list(j2), list(j3), list(j4), list(j5)]
        if unit == 'rad':
            rng = [[degrees(a), degrees(b)] for a, b in rng]
        elif unit != 'deg':
            print('unvalid unit')
            return False
        self.joint_range = rng
        return True

    def get_joint_range(self, unit='deg'):
        if unit == 'deg':
            return [list(r) for r in self.joint_range]
        if unit == 'rad':
            return [[radians(a), radians(b)] for a, b in self.joint_range]
        print('unvalid unit')
        return None

    # -- kinematics ----------------------------------------------------------
    def get_fk(self, pluse):
        """Forward kinematics.

        pluse : iterable of 5 joint angles **in radians** (despite the name;
                the call sites pass transform.pulse2angle(...) output).
        return: [ [x, y, z], geometry_msgs/Quaternion ]  on success,
                None if any joint is outside its range (matching the .so,
                which printed 'i=<v> out of range' only when debug is on).
        """
        ang = [float(a) for a in pluse]
        if len(ang) < 5:
            return None
        names = ('1', 'theta2', '3', '4', '5')   # exact labels recovered from the binary
        for i in range(5):
            deg = degrees(ang[i])
            lo, hi = self.joint_range[i]
            if deg < lo or deg > hi:
                if self.debug:
                    print('%s=%s out of range: [%s, %s]' % (names[i], deg, lo, hi))
                return None

        t1, t2, t3, t4, t5 = ang
        T = (_mdh(0.0, 0.0, t1, 0.0) @
             _mdh(radians(-90), 0.0, t2, 0.0) @
             _mdh(0.0, self.link1, t3, 0.0) @
             _mdh(0.0, self.link2, t4, 0.0) @
             _mdh(radians(-90), 0.0, t5, 0.0))

        R = T[:3, :3]
        approach = R[:, 2]                      # tool points along its z (approach) axis
        pos = T[:3, 3] + approach * (self.link3 + self.tool_link)
        pos = pos + np.array([0.0, 0.0, self.base_link])
        quat = _rot2qua(R)
        if self.debug:
            print('fk position:', pos.tolist())
        return [pos.tolist(), quat]


if __name__ == '__main__':
    fk = ForwardKinematics(debug=True)
    print('links:', fk.get_link())
    print('ranges:', fk.get_joint_range('deg'))
    # home pose: all servos at 500 -> (0, -90, 0, -90, 0) deg
    home = [radians(a) for a in (0, -90, 0, -90, 0)]
    print('home ->', fk.get_fk(home))
