
# Import from custom python packages
from thyroid_ultrasound_robot_control_support.Trajectories.Trajectory import *
from thyroid_ultrasound_robot_control_support.Helpers.vector_magnitude import vector_magnitude


class SimpleTrajectory(Trajectory):

    def __init__(self, distance_between_way_points: float,
                 starting_pose: array,
                 generate_trajectory_on_call: bool = False):

        # Define the parameters of the simple trajectory
        self.starting_pose = starting_pose

        # Call the super class
        super().__init__(distance_between_way_points=distance_between_way_points,
                         generate_trajectory_on_call=generate_trajectory_on_call)

        # Redefine the type of component stored
        self.component_type = WAY_POINT

    def clear(self) -> None:
        """
        Clears the current trajectory from the object and updates the status.

        Returns
        -------
        None
            The next way-point in the trajectory as None.
        """
        # Clear the remaining way-points from the list
        self.components_remaining = []

        # Update the status
        self.status = CLEARED

        # Return the next way-point as None
        return None

    def update(self) -> array:
        """
        Updates the trajectory when the current way-point has been reached.

        Returns
        -------
        Array
            Returns the next point in the trajectory as an array. When the trajectory is complete,
            the next way-point is returned as None.
        """

        try:
            # Add the current way-point to the list of way-points that was reached
            self.components_reached.append(self.components_remaining[0])

            # Remove the first way-point from the list of remaining way-points
            self.components_remaining.pop(0)

            # Update the status of the trajectory
            self.update_status()

            # Return the next remaining way-point
            return self.components_remaining[0]

        # If there are no more points remaining,
        except IndexError:

            # Return None instead
            return None

    def get_current(self) -> array:
        """
        Returns the current point being reached in the trajectory.

        Returns
        -------
        Array
            Returns the next point in the trajectory as an array. If the trajectory has not been generated or is complete,
            the next way-point is returned as None.
        """
        try:
            return self.components_remaining[0]
        except IndexError:

            # Update the status of the trajectory
            self.update_status()

            # Return None instead
            return None


