#!/usr/bin/env python3

"""
File containing the Feature class.
"""

# Import standard python packages
from numpy import array, ndarray, zeros

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.FeatureConstants import *
from thyroid_ultrasound_robot_control_support.Helpers.calc_inverse import calc_inverse
from thyroid_ultrasound_robot_control_support.Helpers.calc_rpy import calc_rpy


class Feature:

    def __init__(self, defining_pose: array = None,
                 translational_locked_axes: tuple = (X_LOCKED, Y_LOCKED, Z_LOCKED),
                 rotational_locked_axes: tuple = (ROLL_LOCKED, PITCH_LOCKED, YAW_LOCKED)):
        """
        Create a feature using a given pose and the axes, both translational and rotational, which should be controlled.

        Parameters
        ----------
        defining_pose
            A valid homogeneous transformation matrix given as 4x4 numpy array
        translational_locked_axes
            A tuple of constants signifying along which translational distance is calculated.
        rotational_locked_axes
            A tuple of constants signifying along which rotational distance is calculated.
        """

        # Ensure that the given pose is an array
        if type(defining_pose) != ndarray:
            defining_pose = array(defining_pose)

        # Ensure the current pose is the correct shape
        if defining_pose.shape != TRANSFORM_MATRIX:
            raise Exception("The shape of the of the current pose, " + str(defining_pose.shape) +
                            ", is not " + str(TRANSFORM_MATRIX) + ".")

        # Save the pose used to define the feature
        self.defining_pose_matrix = defining_pose

        # Calculate the matrix needed to find the error between any pose and the defining pose
        self.inverse_defining_pose_matrix = calc_inverse(defining_pose)

        # Define which translational axes for the axis will be locked
        self.translational_locked_axes = translational_locked_axes

        # Define which rotational axes for the feature will be locked
        self.rotational_locked_axes = rotational_locked_axes

    def distance_to_feature(self, pose_to_measure_against: array) -> dict:
        """
        Calculate the distance between the feature and the pose-to-measure-against.

        Parameters
        ----------
        pose_to_measure_against
            A (4, 4) numpy array containing a valid homogenous transformation matrix.

        Returns
        -------
        dict
            A dictionary containing the translational error w.r.t. the feature frame as a (3, 1) numpy array,
            the translational error w.r.t. the origin frame as a (3, 1) numpy array, and rotational error as
            a (3, 1) numpy array.
        """
        # Ensure that the given pose is an array
        if type(pose_to_measure_against) != ndarray:
            pose_to_measure_against = array(pose_to_measure_against)

        # Ensure the current pose is the correct shape
        if pose_to_measure_against.shape != TRANSFORM_MATRIX:
            raise Exception("The shape of the of the current pose, " + str(pose_to_measure_against.shape) +
                            ", is not " + str(TRANSFORM_MATRIX) + ".")

        # Calculate the transformation between the defining pose and the current pose
        full_error = self.inverse_defining_pose_matrix @ pose_to_measure_against

        # Define a variable to store the error results that are important
        translation_error = zeros(COLUMN_VECTOR)

        # Pull out the error only for the axes that are locked
        for locked_axis in self.translational_locked_axes:
            if locked_axis == X_LOCKED:
                translation_error[X_AXIS] = full_error[X_AXIS][TRANSLATION_COLUMN]
            elif locked_axis == Y_LOCKED:
                translation_error[Y_AXIS] = full_error[Y_AXIS][TRANSLATION_COLUMN]
            elif locked_axis == Z_LOCKED:
                translation_error[Z_AXIS] = full_error[Z_AXIS][TRANSLATION_COLUMN]
            elif locked_axis == X_UNLOCKED or locked_axis == Y_UNLOCKED or locked_axis == Z_UNLOCKED:
                pass
            else:
                raise Exception("Locked axis type of '" + str(locked_axis) + "' was not recognized.")

        # Calculate the roll-pitch-yaw error of the current pose
        full_rpy_error = calc_rpy(full_error[0:3, 0:3])

        # Define a variable to store the error results that are important
        rotation_error = zeros(COLUMN_VECTOR)

        # Pull out the error only for the axes that are locked
        for locked_axis in self.rotational_locked_axes:
            if locked_axis == ROLL_LOCKED:
                rotation_error[X_AXIS] = full_rpy_error[X_AXIS]
            elif locked_axis == PITCH_LOCKED:
                rotation_error[Y_AXIS] = full_rpy_error[Y_AXIS]
            elif locked_axis == YAW_LOCKED:
                rotation_error[Z_AXIS] = full_rpy_error[Z_AXIS]
            elif locked_axis == ROLL_UNLOCKED or locked_axis == PITCH_UNLOCKED or locked_axis == YAW_UNLOCKED:
                pass
            else:
                raise Exception("Locked axis type of '" + str(locked_axis) + "' was not recognized.")

        # Return the error
        return {TRANSLATION_ERROR_WRT_FEATURE: translation_error,
                TRANSLATION_ERROR_WRT_ORIGIN: self.defining_pose_matrix[0:3, 0:3] @ translation_error,
                ROTATIONAL_ERROR: rotation_error}


if __name__ == '__main__':
    test_feature = Feature(defining_pose=array([[0, -1, 0, 1],
                                                [1, 0, 0, 1],
                                                [0, 0, 1, 1],
                                                [0, 0, 0, 1]]),
                           translational_locked_axes=(X_LOCKED, Y_UNLOCKED, Z_UNLOCKED))
    print(test_feature.distance_to_feature(array([[1, 0, 0, 10],
                                                  [0, 1, 0, -10],
                                                  [0, 0, 1, 5],
                                                  [0, 0, 0, 1]])))
