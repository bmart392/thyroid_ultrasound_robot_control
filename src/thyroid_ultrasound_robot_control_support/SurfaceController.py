#!/usr/bin/env python3

"""
File containing the Surface class.
"""

from thyroid_ultrasound_robot_control_support.BasicController import BasicController
from thyroid_ultrasound_robot_control_support.Surface import Surface

# TODO Properly comment this file


class SurfaceController(BasicController):

    def __init__(self, p_gain: float, error_tolerance: float,
                 d_gain: float = None, i_gain: float = None,
                 set_point: Surface = None, min_output: float = None):
        super().__init__(p_gain, error_tolerance,
                         d_gain=d_gain, i_gain=i_gain,
                         set_point=set_point, min_output=min_output)

    def calculate_current_error(self, new_reading):
        if self.set_point is not None:
            return -self.set_point.distance_to_surface(new_reading)