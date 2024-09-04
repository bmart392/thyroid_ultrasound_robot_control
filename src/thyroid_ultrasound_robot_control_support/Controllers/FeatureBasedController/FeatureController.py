"""Defines the FeatureController class"""

# Import standard python packages
from typing import List, Dict, Tuple
from numpy import array, ndarray, append, linspace

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Controllers.BasicController import BasicController
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.Feature import Feature, \
    REFERENCE_FRAME
from thyroid_ultrasound_robot_control_support.Controllers.ControllerConstants import *

# Define a tuple of all the controller names
CONTROLLER_KEYS = (X_LINEAR_CONTROLLER, Y_LINEAR_CONTROLLER, Z_LINEAR_CONTROLLER,
                   X_ANGULAR_CONTROLLER, Y_ANGULAR_CONTROLLER, Z_ANGULAR_CONTROLLER)


class FeatureController:

    def __init__(self, p_gains: List[float], error_tolerances: List[float],
                 d_gains: List[float] = None, i_gains: List[float] = None,
                 min_outputs: List[float] = None,
                 max_outputs: List[float] = None):
        """
        Contains six basic PID controllers whose set-points are determined by 3D feature. Error is defined as the
        difference between the 3D feature and the current pose measured along each of the axes defined in the feature.

        Parameters
        ----------
        p_gains
            The proportional gain for each sub-controller.
        error_tolerances
            The +/- allowance for the error of the system compared to the set-point of each sub-controller.
        i_gains
            The integral gain for each sub-controller.
        d_gains
            The derivative gain for each sub-controller.
        min_outputs
            The minimum output that each sub-controller is allowed to produce.
        max_outputs
            The maximum output that each sub-controller is allowed to produce.
        """

        # Create the basic controller for each axis
        self.controllers = {X_LINEAR_CONTROLLER: BasicController(p_gain=p_gains[X_LINEAR_CONTROLLER],
                                                                 error_tolerance=error_tolerances[X_LINEAR_CONTROLLER],
                                                                 i_gain=i_gains[X_LINEAR_CONTROLLER],
                                                                 d_gain=d_gains[X_LINEAR_CONTROLLER],
                                                                 min_output=min_outputs[X_LINEAR_CONTROLLER],
                                                                 max_output=max_outputs[X_LINEAR_CONTROLLER]),
                            Y_LINEAR_CONTROLLER: BasicController(p_gain=p_gains[Y_LINEAR_CONTROLLER],
                                                                 error_tolerance=error_tolerances[Y_LINEAR_CONTROLLER],
                                                                 i_gain=i_gains[Y_LINEAR_CONTROLLER],
                                                                 d_gain=d_gains[Y_LINEAR_CONTROLLER],
                                                                 min_output=min_outputs[Y_LINEAR_CONTROLLER],
                                                                 max_output=max_outputs[Y_LINEAR_CONTROLLER]),
                            Z_LINEAR_CONTROLLER: BasicController(p_gain=p_gains[Z_LINEAR_CONTROLLER],
                                                                 error_tolerance=error_tolerances[Z_LINEAR_CONTROLLER],
                                                                 i_gain=i_gains[Z_LINEAR_CONTROLLER],
                                                                 d_gain=d_gains[Z_LINEAR_CONTROLLER],
                                                                 min_output=min_outputs[Z_LINEAR_CONTROLLER],
                                                                 max_output=max_outputs[Z_LINEAR_CONTROLLER]),
                            X_ANGULAR_CONTROLLER: BasicController(p_gain=p_gains[X_ANGULAR_CONTROLLER],
                                                                  error_tolerance=error_tolerances[
                                                                      X_ANGULAR_CONTROLLER],
                                                                  i_gain=i_gains[X_ANGULAR_CONTROLLER],
                                                                  d_gain=d_gains[X_ANGULAR_CONTROLLER],
                                                                  min_output=min_outputs[X_ANGULAR_CONTROLLER],
                                                                  max_output=max_outputs[X_ANGULAR_CONTROLLER]),
                            Y_ANGULAR_CONTROLLER: BasicController(p_gain=p_gains[Y_ANGULAR_CONTROLLER],
                                                                  error_tolerance=error_tolerances[
                                                                      Y_ANGULAR_CONTROLLER],
                                                                  i_gain=i_gains[Y_ANGULAR_CONTROLLER],
                                                                  d_gain=d_gains[Y_ANGULAR_CONTROLLER],
                                                                  min_output=min_outputs[Y_ANGULAR_CONTROLLER],
                                                                  max_output=max_outputs[Y_ANGULAR_CONTROLLER]),
                            Z_ANGULAR_CONTROLLER: BasicController(p_gain=p_gains[Z_ANGULAR_CONTROLLER],
                                                                  error_tolerance=error_tolerances[
                                                                      Z_ANGULAR_CONTROLLER],
                                                                  i_gain=i_gains[Z_ANGULAR_CONTROLLER],
                                                                  d_gain=d_gains[Z_ANGULAR_CONTROLLER],
                                                                  min_output=min_outputs[Z_ANGULAR_CONTROLLER],
                                                                  max_output=max_outputs[Z_ANGULAR_CONTROLLER])}

        # Define variables for storing a history of the last error points
        self.error_history_length = 15
        self.error_history = array([])

        # Define a variable to store the feature set-point
        self.feature_set_point: Feature = None

    def update_set_point(self, new_feature_set_point: Feature) -> None:
        """
        Updates the set-point of each sub-controller depending on if each axis is locked or unlocked.

        Parameters
        ----------
        new_feature_set_point
            The new feature to save as the set-point for the controller.
        """

        # Save the new feature
        self.feature_set_point = new_feature_set_point

        # Convert the feature into set points for each of the controllers, based on whether that feature is locked
        for controller_key, axis_status, individual_set_point in zip(CONTROLLER_KEYS,
                                                                     new_feature_set_point.status_of_axes,
                                                                     [0] * 6):
            # If the axis is locked, set the appropriate set point for the appropriate controller
            if axis_status:
                self.controllers[controller_key].update_set_point(individual_set_point)
            # Otherwise clear the set point
            else:
                self.controllers[controller_key].update_set_point(None)

    def set_gain(self, controller_selector: int, channel_selector: int, new_gain_value: float):
        """
        Updates the selected gain of the controller to the new value.

        Parameters
        ----------
        controller_selector
            The controller whose gains should be changed.
        channel_selector
            The gain that should be updated.
        new_gain_value
            The new value to set the gain to.
        """
        try:
            # Update the P gain if selected
            if channel_selector == P_GAIN:
                self.controllers[controller_selector].p_gain = new_gain_value

            # Update the I gain if selected
            elif channel_selector == I_GAIN:
                self.controllers[controller_selector].i_gain = new_gain_value

            # Update the D gain if selected
            elif channel_selector == D_GAIN:
                self.controllers[controller_selector].d_gain = new_gain_value

            # Raise an exception if the gain given is not recognized
            else:
                raise Exception("Incorrect channel selected.")
        except IndexError:
            raise Exception("Controller selector of " + str(controller_selector) + " was not recognized.")

    def calculate_output(self, given_pose: ndarray) -> Tuple[Dict[int, Tuple[float, bool, float]], bool,
                                                             Dict[int, Tuple[float, bool, float]]]:
        """
        Calculates the control output of each controller based on the given pose.
        Also returns if the set point has been reached and the current error of the system.

        Parameters
        ----------
        given_pose
            The pose received by the controller formatted as a homogeneous transformation matrix.

        Returns
        -------
        tuple
            Returns a tuple containing: a dictionary containing the output value from each sub-controller,
            a boolean noting if the feature has been reached according to the results from each sub-controller,
            and a dictionary containing the error values from each sub-controller
        """

        # Calculate the error between the given 
        translation_error, rotation_error, result_reference_frame = self.feature_set_point.distance_to_reference_pose(
            reference_pose=given_pose,
            result_reference_frame=REFERENCE_FRAME)

        # Define variables for storing the values to return from the function
        output_values_to_return = {}
        combined_success_value = True
        error_values_to_return = {}

        # For each controller and new reading,
        for controller_key, new_value in zip(CONTROLLER_KEYS, append(translation_error, rotation_error)):

            # Calculate the output value, success result, and current error value
            output_values_to_return[controller_key], temp_success_result, error_values_to_return[controller_key] = \
                self.controllers[controller_key].calculate_output(new_value)

            # If the controller does not have a set_point, overwrite the error success result as True
            if self.controllers[controller_key].set_point is None:
                temp_success_result = True

            # Calculate the overall success result
            combined_success_value = combined_success_value and temp_success_result

        return output_values_to_return, combined_success_value, error_values_to_return


if __name__ == '__main__':
    # Define the number of poses to create
    num_points = 10

    # Define the X, Y, and Z values to create
    x_values = linspace(10, 0, num_points + 1)
    y_values = linspace(5, 0, num_points + 1)
    z_values = linspace(-10, 0, num_points + 1)

    # Create a test controller
    test_controller = FeatureController(p_gains=[1] * 6,
                                        error_tolerances=[1.5] * 6,
                                        d_gains=[0] * 6,
                                        i_gains=[0] * 6,
                                        min_outputs=[0] * 6,
                                        max_outputs=[10000] * 6)

    # Update the set point on the controller
    test_controller.update_set_point(Feature(defining_pose=array([[1, 0, 0, 0],
                                                                  [0, 1, 0, 0],
                                                                  [0, 0, 1, 0],
                                                                  [0, 0, 0, 1]]),
                                             status_of_axes=(True, True, True, False, False, False)))

    for x, y, z in zip(x_values, y_values, z_values):
        temp_pose = array([[1, 0, 0, x],
                           [0, 1, 0, y],
                           [0, 0, 1, z],
                           [0, 0, 0, 1]])

        results = test_controller.calculate_output(temp_pose)

        heading = 'Feature Set Point Reached? ' + str(results[1])
        print('-' * len(heading))
        print(heading)
        print('-' * len(heading))
        for key in CONTROLLER_KEYS:

            print('Axis ' + '{:01}'.format(key) + ' - Output: ' + '{:05}'.format(results[0][key]) +
                  ' - Error: ' + '{:05}'.format(results[2][key]))
