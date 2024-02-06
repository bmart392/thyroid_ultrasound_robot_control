

from thyroid_ultrasound_robot_control_support.Controllers.BasicController import BasicController
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.Feature import Feature, \
    TRANSLATION_ERROR_WRT_FEATURE, TRANSLATION_ERROR_WRT_ORIGIN, ROTATIONAL_ERROR
from thyroid_ultrasound_robot_control_support.Helpers.vector_magnitude import vector_magnitude


class FeatureController(BasicController):

    def __init__(self, p_gain: float, error_tolerance: float,
                 d_gain: float = None, i_gain: float = None,
                 set_point: Feature = None, min_output: float = None,
                 max_output: float = None):
        """
        Creates a basic PID controller where the set-point is a 3D feature. Error is defined as the distance away from
        the surface measured along each of the axes defined in the feature. Inherits basic functionality from the
        BasicController class.

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

        super().__init__(p_gain, error_tolerance,
                         d_gain=d_gain, i_gain=i_gain,
                         set_point=set_point, min_output=min_output,
                         max_output=max_output)

    def calculate_current_error(self, new_reading) -> float:
        """
        This function is not implemented for feature-based controllers.
        """
        raise Exception("This method is not implemented for feature-based controllers.")

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
