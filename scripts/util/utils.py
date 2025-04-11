import math

import numpy as np


def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    x = cy * cp * cr + sy * sp * sr
    y = cy * cp * sr - sy * sp * cr
    z = sy * cp * sr + cy * sp * cr
    w = sy * cp * cr - cy * sp * sr
    return x, y, z, w


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def R(x, y):
    """
    Polar transform (r, theta) for (x, y)
    """
    r = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)
    return r, theta


def M(theta):
    """
    Map theta to [-pi, pi)
    """
    theta %= 2 * math.pi
    if theta < -math.pi: theta += 2 * math.pi
    elif theta >= math.pi: theta -= 2 * math.pi
    return theta


def normalize_basis(p1, p2):
    """
    Normalize the basis so that p1 equals (0, 0, 0)
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    phi1 = p1[2]
    # Rotation matrix with -phi
    x_new = math.cos(phi1) * dx + math.sin(phi1) * dy
    y_new = -math.sin(phi1) * dx + math.cos(phi1) * dy
    phi_new = p2[2] - p1[2]
    return x_new, y_new, phi_new


def rad2deg(rad):
    return rad / math.pi * 180