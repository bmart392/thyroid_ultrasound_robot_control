
# Import standard packages
from numpy import ndarray
from typing import Tuple

# Import custom python packages
from calc_rpy import calc_rpy


def calc_rpy_and_point_from_transform(transformation: ndarray) -> Tuple[Tuple[float], ndarray]:
    """
    Converts a pose message to equivalent roll, pitch, yaw (RPY) values and 3D cartesian position.

    Parameters
    ----------
    transformation
        A homogeneous transformation matrix formatted as a 4x4 array

    Returns
    -------
    A tuple containing two objects, the first being the orientation of the pose as RPY values (in that order) and
    the second being a numpy array of the 3D cartesian position of the pose.

    """

    return calc_rpy(transformation[0:3, 0:3]), transformation[0:3, 3]
