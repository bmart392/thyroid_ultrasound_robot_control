#!/usr/bin/env python3

"""
File containing the Basic Controller class.
"""

# Import standard python packages
from numpy import array, append, delete

# Import custom python packages
from thyroid_ultrasound_robot_control_support.ControllerConstants import *

# TODO Comment this file


class BasicController:

    def __init__(self, p_gain: float, error_tolerance: float,
                 d_gain: float = None, i_gain: float = None,
                 set_point=None, min_output: float = None):
        """
        Creates a basic set-point based PID controller.
        """

        self.error_history_length = 15
        self.error_history = array([])

        self.p_gain = p_gain
        self.d_gain = d_gain
        self.i_gain = i_gain

        self.set_point = set_point
        self.min_output = min_output

        self.error_tolerance = error_tolerance

    def update_set_point(self, new_set_point) -> None:
        self.set_point = new_set_point

    def set_gain(self, channel_selector: int, new_gain_value: float):

        if channel_selector == P_GAIN:
            self.p_gain = new_gain_value
        elif channel_selector == I_GAIN:
            self.i_gain = new_gain_value
        elif channel_selector == D_GAIN:
            self.d_gain = new_gain_value
        else:
            raise Exception("Incorrect channel selected.")

    def calculate_current_error(self, new_reading):
        """
        Calculates the current error based on the new reading.
        """
        return self.set_point - new_reading

    def calculate_output(self, new_reading):

        output = 0.

        if self.set_point is not None:

            # Calculate the current error
            current_error = self.calculate_current_error(new_reading)

            # Calculate the output based on the proportional gain
            output = output + (self.p_gain * current_error)

            if len(self.error_history) > 0 and self.p_gain is not None:
                # Calculate the output based on the derivative gain
                output = output + (self.d_gain * (current_error - self.error_history[-1]))

            if len(self.error_history) > 1 and self.i_gain is not None:
                # Calculate the output based on the integral gain
                output = output + (self.i_gain * sum(self.error_history))

            # Update the error history with the new error
            self.error_history = append(self.error_history, current_error)

            # Make sure that the error history is not too long
            if len(self.error_history) > self.error_history_length:
                self.error_history = delete(self.error_history, 0)

            if self.min_output is not None:
                if output < self.min_output:
                    output = 0

            return output, abs(current_error) <= self.error_tolerance, current_error

        return output, False, 0


