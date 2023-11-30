#!/usr/bin/env python3

"""
File containing the Surface class.
"""

from thyroid_ultrasound_robot_control_support.BasicController import BasicController
from thyroid_ultrasound_robot_control_support.Surface import Surface


class SurfaceController(BasicController):

    def __init__(self, p_gain: float, error_tolerance: float,
                 d_gain: float = None, i_gain: float = None,
                 set_point: Surface = None, min_output: float = None,
                 max_output: float = None):
        """
        Creates a basic PID controller where the set-point is a 3D surface. Error is defined as the distance away from
        the surface measured normal to the surface. Inherits basic functionality from the BasicController class.

        Parameters
        ----------
        p_gain
            The proportional gain for the controller.
        error_tolerance
            The +/- allowance for the error of the system compared to the set-point.
        i_gain
            The integral gain for the controller.
        d_gain
            The derivative gain for the controller.
        set_point
            The surface that the controller is attempting to reach.
        min_output
            The minimum output that the controller is allowed to produce.
        max_output
            The maximum output that the controller is allowed to produce.
        """

        # Call the init method of the BasicController class
        super().__init__(p_gain, error_tolerance,
                         d_gain=d_gain, i_gain=i_gain,
                         set_point=set_point, min_output=min_output,
                         max_output=max_output)

    def calculate_current_error(self, new_reading):
        """
        Calculates the error as the distance from the set-point to the surface as measured along
        the normal vector of the surface.

        Parameters
        ----------
        new_reading
            A position in 3D space expressed as (x, y, z) vector.
        """
        return -self.set_point.distance_to_surface(new_reading)
