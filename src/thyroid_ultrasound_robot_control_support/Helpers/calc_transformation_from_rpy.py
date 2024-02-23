from math import cos, sin
from numpy import array, ndarray


def calc_transformation_from_rpy(xyz: tuple, rpy: tuple) -> ndarray:
    """
    Calculates the equivalent transformation matrix from a set of roll, pitch, and yaw angles and x, y, and z positions.
    Parameters
    ----------
    xyz
        A tuple containing an x, y, and z position.
    rpy
        A tuple containing roll, pitch, and yaw angles.

    Returns
    -------
    ndarray
        An array containing a homogeneous transformation matrix.
    """
    # Define variables for clarity
    c_g = cos(rpy[0])
    c_b = cos(rpy[1])
    c_a = cos(rpy[2])
    s_g = sin(rpy[0])
    s_b = sin(rpy[1])
    s_a = sin(rpy[2])
    return array([
        [c_a * c_b, c_a * s_b * s_g - s_a * c_g, c_a * s_b * c_g + s_a * s_g, xyz[0]],
        [s_a * c_b, s_a * s_b * s_g + c_a * c_g, s_a * s_b * c_g - c_a * s_g, xyz[1]],
        [-s_b, c_b * s_g, c_b * c_g, xyz[2]],
        [0, 0, 0, 1],
    ])
