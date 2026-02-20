import numpy as np
import math


def rotation_matrix_y(angle_degrees):
    theta = np.radians(angle_degrees)
    return np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [ 0,             1, 0            ],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def rotation_matrix_z(angle_degrees):
    theta = np.radians(angle_degrees)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,             0,              1]
    ])


def rotation_matrix_x(angle_degrees):
    theta = np.radians(angle_degrees)
    return np.array([
        [1, 0,            0           ],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta),  np.cos(theta)]
    ])


def wrap_angle_pi(angle):
    """
    Wrap angle to [-π, π)
    """
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    return angle
