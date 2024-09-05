"""
Contains the code for the PoseToPoseTrajectory class.
"""

# TODO - Low - Properly comment this file
# TODO - Low - Properly implement this class

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Trajectories.SimpleTrajectories.SimpleTrajectory import *
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.Feature import Feature
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.FeatureConstants import *


class PoseToPoseTrajectory(SimpleTrajectory):

    def __init__(self, distance_between_way_points: float,
                 starting_pose: array,
                 ending_pose: array,
                 generate_trajectory_on_call: bool = True):

        # Define the ending offset needed to make the trajectory
        self.ending_pose = ending_pose

        super().__init__(distance_between_way_points=distance_between_way_points,
                         starting_pose=starting_pose,
                         generate_trajectory_on_call=generate_trajectory_on_call)

    def generate(self) -> Feature:
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
            if self.ending_pose.shape != (4, 4):
                raise Exception("The shape of the end pose array is " + str(self.ending_pose.shape) + " instead"
                                                                                                      " of 4x4.")

            # Calculate the number of points to generate along the path
            num_points = abs(
                int(ceil(vector_magnitude(self.ending_pose[0:3, 3] - self.starting_pose[0:3, 3]) /
                         self.distance_between_way_points))) + 1

            # Interpolate poses between the starting and ending poses
            intermediate_poses = linspace(self.starting_pose, self.ending_pose, num_points)

            # Create a feature for each pose in the trajectory
            for ii in range(num_points):
                self.components_in_trajectory.append(
                    Feature(defining_pose=intermediate_poses[ii],
                            status_of_axes=tuple([LOCKED] * 6)))

        else:
            raise Exception("The ending-pose cannot be None.")

        # Copy the way-points in the trajectory over to the points remaining
        self.components_remaining = deepcopy(self.components_in_trajectory)

        # Update the status of the trajectory
        self.status = GENERATED

        # Return the first way-point in the trajectory
        return self.components_remaining[0]


if __name__ == '__main__':
    # Create the trajectory object
    temp_trajectory = PoseToPoseTrajectory(distance_between_way_points=.1,
                                           starting_pose=zeros((4, 4)),
                                           ending_pose=array([[0, 0, 0, 5],
                                                              [0, 0, 0, -2],
                                                              [0, 0, 0, 4],
                                                              [0, 0, 0, 0]]),
                                           generate_trajectory_on_call=False)

    # Check the status of the trajectory
    print("Current status: " + temp_trajectory.status)
    print("Number of points in the trajectory: " + str(len(temp_trajectory.components_in_trajectory)))
    print("Number of points remaining in the trajectory: " + str(len(temp_trajectory.components_remaining)))
    print("Number of points reached in the trajectory: " + str(len(temp_trajectory.components_reached)))
    print("---")

    # Generate the trajectory
    temp_trajectory.generate()

    # Check the status of the trajectory
    print("Current status: " + temp_trajectory.status)
    print("Number of points in the trajectory: " + str(len(temp_trajectory.components_in_trajectory)))
    print("Number of points remaining in the trajectory: " + str(len(temp_trajectory.components_remaining)))
    print("Number of points reached in the trajectory: " + str(len(temp_trajectory.components_reached)))
    print("---")

    # Update the trajectory
    temp_trajectory.update()

    # Check the status of the trajectory
    print("Current status: " + temp_trajectory.status)
    print("Number of points in the trajectory: " + str(len(temp_trajectory.components_in_trajectory)))
    print("Number of points remaining in the trajectory: " + str(len(temp_trajectory.components_remaining)))
    print("Number of points reached in the trajectory: " + str(len(temp_trajectory.components_reached)))
    print("---")

    # Clear the trajectory
    temp_trajectory.update()

    # Check the status of the trajectory
    print("Current status: " + temp_trajectory.status)
    print("Number of points in the trajectory: " + str(len(temp_trajectory.components_in_trajectory)))
    print("Number of points remaining in the trajectory: " + str(len(temp_trajectory.components_remaining)))
    print("Number of points reached in the trajectory: " + str(len(temp_trajectory.components_reached)))
    print("---")

    # Clear the trajectory
    temp_trajectory.update()

    # Check the status of the trajectory
    print("Current status: " + temp_trajectory.status)
    print("Number of points in the trajectory: " + str(len(temp_trajectory.components_in_trajectory)))
    print("Number of points remaining in the trajectory: " + str(len(temp_trajectory.components_remaining)))
    print("Number of points reached in the trajectory: " + str(len(temp_trajectory.components_reached)))
    print("---")

    # Clear the trajectory
    temp_trajectory.update()

    # Check the status of the trajectory
    print("Current status: " + temp_trajectory.status)
    print("Number of points in the trajectory: " + str(len(temp_trajectory.components_in_trajectory)))
    print("Number of points remaining in the trajectory: " + str(len(temp_trajectory.components_remaining)))
    print("Number of points reached in the trajectory: " + str(len(temp_trajectory.components_reached)))
    print("---")
