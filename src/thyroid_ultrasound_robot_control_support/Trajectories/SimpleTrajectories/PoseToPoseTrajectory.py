"""
Contains the code for the PoseToPoseTrajectory class.
"""

# TODO - Low - Properly comment this file
# TODO - Low - Properly implement this class

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Trajectories.SimpleTrajectories.SimpleTrajectory import *


class PoseToPoseTrajectory(SimpleTrajectory):

    def __init__(self, distance_between_way_points: float,
                 starting_pose: array,
                 ending_pose: array = None,
                 generate_trajectory_on_call: bool = True):

        # Define the ending offset needed to make the trajectory
        self.ending_pose = ending_pose

        super().__init__(distance_between_way_points=distance_between_way_points,
                         starting_pose=starting_pose,
                         generate_trajectory_on_call=generate_trajectory_on_call)

    def generate(self) -> array:
        """
        Generates a trajectory between the starting-pose and the ending-pose.

        Returns
        -------
        array
            The first way-point in the trajectory.
        """

        # If an ending pose was given,
        if self.ending_pose is not None:

            # If the pose was not given as an array,
            if type(self.ending_pose) != array:
                # Make it into an array
                self.ending_pose = array(self.ending_pose)

            # Ensure that the array is 4x4,
            if self.ending_pose.shape == (4, 4):

                # Calculate the number of points to generate along the path
                num_points = abs(
                    int(ceil(vector_magnitude(self.ending_pose[0:3, 3] - self.starting_pose[0:3, 3]) /
                             self.distance_between_way_points))) + 1

                raise Exception("Moving pose to pose has not been implemented.")

            else:
                raise Exception("The shape of the end pose array is " + str(self.ending_pose.shape) + " instead"
                                    " of 4x4.")

        else:
            raise Exception("The ending-pose cannot be None.")