"""
Contains the code for the RotationTrajectory class.
"""

# TODO - Low - Properly comment this file
# TODO - Low - Properly implement this class

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Trajectories.SimpleTrajectories.SimpleTrajectory import *


class RotationTrajectory(SimpleTrajectory):

    def __init__(self, distance_between_way_points: float,
                 starting_pose: array,
                 ending_offset_rotation: array = None,
                 generate_trajectory_on_call: bool = True):
        """

        Parameters
        ----------
        ending_offset_rotation
            The rotation offset from the starting pose at which to define the goal pose.
        """

        # Define the ending offset needed to make the trajectory
        self.ending_offset_rotation = ending_offset_rotation

        # Call the init function of the super class
        super().__init__(distance_between_way_points=distance_between_way_points,
                         starting_pose=starting_pose,
                         generate_trajectory_on_call=generate_trajectory_on_call)

    def generate(self) -> array:
        """
        Generates a trajectory between the starting-pose and a pose the ending-offset rotation away.

        Returns
        -------
        array
            The first way-point in the trajectory.
        """

        # If an ending offset was given,
        if self.ending_offset_distance is not None:

            # If the offset was not given as an array,
            if type(self.ending_offset_distance) != array:

                # Change it into an array
                self.ending_offset_distance = array(self.ending_offset_distance)

            # If the array is a row vector of the correct size,
            if self.ending_offset_distance.shape == (3, ):

                # Change it into a column vector of the right size
                self.ending_offset_distance = self.ending_offset_distance.reshape((3, 1))

            # If the array is not a column vector of the right size,
            elif self.ending_offset_distance.shape != (3, 1):
                raise Exception("The shape of the ending offset was " + str(self.ending_offset_distance.shape) +
                                " instead of 3x1 or 1x3.")

            # Define the distance to travel and the number of points to generate along the way
            num_points = abs(
                int(ceil(vector_magnitude(self.ending_offset_distance) / self.distance_between_way_points))) + 1

            # Interpolate a set of column vectors between 0 and the ending offset distance
            trajectory = array([linspace(start=0, stop=self.ending_offset_distance[0], num=num_points),
                                linspace(start=0, stop=self.ending_offset_distance[1], num=num_points),
                                linspace(start=0, stop=self.ending_offset_distance[2], num=num_points)])

            # For each column vector in the trajectory
            for ii in range(trajectory.shape[1]):

                # Create a zero rotation and translation homogeneous transformation matrix
                temp_transformation_matrix = zeros((4, 4))

                # Add in the translation from the current column vector
                temp_transformation_matrix[0:3, 3, newaxis] = trajectory[:, ii]

                # Add a new pose to the trajectory of the starting pose plus the new transformation
                self.components_in_trajectory.append(self.starting_pose + temp_transformation_matrix)

        else:
            raise Exception("The ending-offset-distance cannot be None.")

        # Copy the way-points in the trajectory over to the points remaining
        self.components_remaining = deepcopy(self.components_in_trajectory)

        # Update the status of the trajectory
        self.status = GENERATED

        # Return the first way-point in the trajectory
        return self.components_remaining[0]