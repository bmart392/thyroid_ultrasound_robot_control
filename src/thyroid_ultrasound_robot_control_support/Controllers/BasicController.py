#!/usr/bin/env python3

"""
File containing the Basic Controller class.
"""

# Import standard python packages
from numpy import array, append, delete

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Controllers.ControllerConstants import *


class BasicController:

    def __init__(self, p_gain: float, error_tolerance: float,
                 i_gain: float = None, d_gain: float = None,
                 set_point=None, min_output: float = None,
                 max_output: float = None):
        """
        Creates a basic set-point based PID controller.

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
            The set-point that the controller is attempting to reach.
        min_output
            The minimum output that the controller is allowed to produce.
        max_output
            The maximum output that the controller is allowed to produce.
        """

        # Define variables for storing a history of the last error points
        self.error_history_length = 15
        self.error_history = array([])

        # Save the controller gains
        self.p_gain = p_gain
        self.d_gain = d_gain
        self.i_gain = i_gain

        # Save the set point for the controller
        self.set_point = set_point

        # Save the minimum and maximum output
        self.min_output = min_output
        self.max_output = max_output

        # Save the error tolerance
        self.error_tolerance = error_tolerance

    def update_set_point(self, new_set_point) -> None:
        """
        Updates the set-point of the controller.

        Parameters
        ----------
        new_set_point
            The new value to save as the set-point for the controller.
        """
        self.set_point = new_set_point

    def set_gain(self, channel_selector: int, new_gain_value: float):
        """
        Updates the selected gain of the controller to the new value.

        Parameters
        ----------
        channel_selector
            The gain that should be updated.
        new_gain_value
            The new value to set the gain to.
        """

        # Update the P gain if selected
        if channel_selector == P_GAIN:
            self.p_gain = new_gain_value

        # Update the I gain if selected
        elif channel_selector == I_GAIN:
            self.i_gain = new_gain_value

        # Update the D gain if selected
        elif channel_selector == D_GAIN:
            self.d_gain = new_gain_value

        # Raise an exception if the gain given is not recognized
        else:
            raise Exception("Incorrect channel selected.")

    def calculate_current_error(self, new_reading) -> float:
        """
        Calculates the current error based on the new reading.

        Parameters
        ----------
        new_reading
            The new error reading received by the controller.
        """
        # Calculate the current error
        return self.set_point - new_reading

    def calculate_output(self, new_reading) -> (float, bool, float):
        """
        Calculates the control output based on the current error. Also returns if the set point has been reached and
        the current error of the system.

        Parameters
        ----------
        new_reading
            The new error reading of the system.
        """

        # Define the output result
        output = 0.

        # Only calculate an output if the set-point has been defined for the controller
        if self.set_point is not None:

            # Calculate the current error
            current_error = self.calculate_current_error(new_reading)

            # If the current error is greater than the error tolerance,
            if abs(current_error) > self.error_tolerance:

                # Calculate the output based on the proportional gain
                output = output + (self.p_gain * current_error)

                # If the previous error is known and the D gain is set
                if len(self.error_history) > 0 and self.d_gain is not None:
                    # Calculate the output based on the derivative gain
                    output = output + (self.d_gain * (current_error - self.error_history[-1]))

                # If the previous error is known and the I gain is set
                if len(self.error_history) > 1 and self.i_gain is not None:
                    # Calculate the output based on the integral gain
                    output = output + (self.i_gain * sum(self.error_history))

                # Update the error history with the new error
                self.error_history = append(self.error_history, current_error)

                # Make sure that the error history is not too long
                if len(self.error_history) > self.error_history_length:
                    self.error_history = delete(self.error_history, 0)

                # Only if the min output has been set
                if self.min_output is not None:

                    # If the output is smaller than the min_output
                    if abs(output) < self.min_output:

                        # Make the output the min output of the system
                        output = self.min_output * abs(output)/output

                # Only if the max output has been set
                if self.max_output is not None:

                    # If the output is larger than the max output
                    if abs(output) > self.max_output:

                        # Make the output the max output of the system
                        output = self.max_output * abs(output)/output

                # Return the calculated output, that the set point has not been reached, and the current error
                return output, False, current_error

            else:

                # Return the zero output, that the set point has been reached, and the current error
                return output, True, current_error

        # Return the zero output, that the set point has not been reached, and zero error
        return output, False, 0


