#!/usr/bin/env python3

"""
File containing the Feature class.
"""

# Import standard python packages
from numpy import array, ndarray, zeros
from typing import Tuple

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.FeatureConstants import *
from thyroid_ultrasound_robot_control_support.Helpers.calc_inverse import calc_inverse
from thyroid_ultrasound_robot_control_support.Helpers.calc_rpy import calc_rpy
from thyroid_ultrasound_support.MessageConversion.convert_array_to_float64_multi_array_message import \
    convert_array_to_float64_multi_array_message
from thyroid_ultrasound_support.MessageConversion.convert_float64_multi_array_message_to_array import \
    convert_float64_multi_array_message_to_array

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import FeatureObjectMsg


class Feature:

    def __init__(self, defining_pose: array = None,
                 status_of_axes: tuple = (LOCKED, UNLOCKED, UNLOCKED, LOCKED, UNLOCKED, UNLOCKED),
                 feature_as_msg: FeatureObjectMsg = None):
        """
        Create a feature using a given pose and the axes, both translational and rotational, which should be controlled.

        Parameters
        ----------
        defining_pose
            A valid homogeneous transformation matrix given as 4x4 numpy array
        status_of_axes
            A tuple of 6 boolean values signifying which axes should have motion locked.
            The order of parameters is: X, Y, Z, Roll, Pitch, Yaw
        """

        # If a message of the correct type is provided, use the message to build the object
        if feature_as_msg is not None and type(feature_as_msg) == FeatureObjectMsg:

            # Pull the pose matrix out of the message
            self.feature_origin = convert_float64_multi_array_message_to_array(feature_as_msg.origin)

            # Pull out the status of each translational axis
            self.status_of_axes = (feature_as_msg.lin_x_status, feature_as_msg.lin_y_status,
                                   feature_as_msg.lin_z_status, feature_as_msg.ang_x_status,
                                   feature_as_msg.ang_y_status, feature_as_msg.ang_z_status)

        # Otherwise use the other data provided,
        else:

            # Ensure that the given pose is an array
            if type(defining_pose) != ndarray:
                defining_pose = array(defining_pose)

            # Ensure the current pose is the correct shape
            if defining_pose.shape != TRANSFORM_MATRIX:
                raise Exception("The shape of the of the current pose, " + str(defining_pose.shape) +
                                ", is not " + str(TRANSFORM_MATRIX) + ".")

            if len(status_of_axes) != 6:
                raise Exception("The number of axis status given is not sufficient. Only " + str(len(status_of_axes)) +
                                " statuses were given.")

            # Save the pose used to define the feature
            self.feature_origin = defining_pose

            # Save the motion status for each axis
            self.status_of_axes = status_of_axes

        # Calculate the matrix needed to find the error between any pose and the defining pose
        self.inverse_of_feature_origin = calc_inverse(self.feature_origin)

    def distance_to_reference_pose(self, reference_pose: array,
                                   result_reference_frame: str = FEATURE_FRAME) -> Tuple:
        """
        Calculates the distance between the feature origin and the reference pose.

        Parameters
        ----------
        reference_pose
            A (4, 4) numpy array containing a valid homogenous transformation matrix.
        result_reference_frame
            A string signifying which reference should be used to calculate the result distance

        Returns
        -------
        tuple
            A tuple containing the translational error as a numpy array and the rotational error as a
            numpy array both w.r.t. the given reference frame. In addition, the reference frame selected will be
            returned.
        """
        # Ensure that the given pose is an array
        if type(reference_pose) != ndarray:
            reference_pose = array(reference_pose)

        # Ensure the current pose is the correct shape
        if reference_pose.shape != TRANSFORM_MATRIX:
            raise Exception("The shape of the of the current pose, " + str(reference_pose.shape) +
                            ", is not " + str(TRANSFORM_MATRIX) + ".")

        # Calculate the transformation between the feature origin and the reference pose
        # according to the given reference frame
        if result_reference_frame == FEATURE_FRAME:
            full_error = self.inverse_of_feature_origin @ reference_pose
        elif result_reference_frame == REFERENCE_FRAME:
            full_error = calc_inverse(reference_pose) @ self.feature_origin
        else:
            raise Exception("The given reference frame of " + str(result_reference_frame) + " is not recognized.")

        # Define arrays to store the error results that are important
        result_translation_error = zeros(3)
        result_rotation_error = zeros(3)

        # Calculate the roll-pitch-yaw error of the current pose
        full_rpy_error = calc_rpy(full_error[0:3, 0:3])

        # Pull out the translation error for ease of use
        full_translation_error = full_error[0:3, TRANSLATION_COLUMN]

        # Pull out the error only for the axes that are locked
        for result_error, calculated_error in zip((result_translation_error, result_rotation_error),
                                                  (full_translation_error, full_rpy_error), ):
            for index in range(3):
                if self.status_of_axes[index]:
                    result_error[index] = calculated_error[index]

        # Return the error
        return result_translation_error, result_rotation_error, result_reference_frame

    def to_msg(self) -> FeatureObjectMsg:
        """Creates a FeatureObjectMsg from the data stored within the object"""

        # Create a new empty object to return
        new_msg = FeatureObjectMsg()

        # Fill in the data for the object
        new_msg.origin = convert_array_to_float64_multi_array_message(self.feature_origin)
        new_msg.lin_x_status = self.status_of_axes[LIN_X]
        new_msg.lin_y_status = self.status_of_axes[LIN_Y]
        new_msg.lin_z_status = self.status_of_axes[LIN_Z]
        new_msg.ang_x_status = self.status_of_axes[ROLL_X]
        new_msg.ang_y_status = self.status_of_axes[PITCH_Y]
        new_msg.ang_z_status = self.status_of_axes[YAW_Z]

        # Return the message
        return new_msg


if __name__ == '__main__':
    test_feature = Feature(defining_pose=array([[1, 0, 0, 1],
                                                [0, 1, 0, 1],
                                                [0, 0, 1, 1],
                                                [0, 0, 0, 1]]),
                           status_of_axes=(LOCKED, LOCKED, LOCKED, LOCKED, LOCKED, LOCKED))
    for reference_frame in (FEATURE_FRAME, REFERENCE_FRAME):
        results = test_feature.distance_to_reference_pose(reference_pose=array([[0, -1, 0, 10],
                                                                                [1, 0, 0, -10],
                                                                                [0, 0, 1, 5],
                                                                                [0, 0, 0, 1]]),
                                                          result_reference_frame=reference_frame)
        heading = 'Translation Error ' + reference_frame + ':'
        print('-' * len(heading))
        print(heading)
        print('-' * len(heading))
        print('X: ' + str(results[0][0]))
        print('Y: ' + str(results[0][1]))
        print('Z: ' + str(results[0][2]))
        heading = 'Rotation Error ' + reference_frame + ':'
        print('-' * len(heading))
        print(heading)
        print('-' * len(heading))
        print('X: ' + str(results[1][0]))
        print('Y: ' + str(results[1][1]))
        print('Z: ' + str(results[1][2]))
