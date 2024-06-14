"""
Contains code for the Trajectory class and import statements and constants used in all trajectories.
"""

# TODO - Low - Properly comment this file
# TODO - Low - When I want to udpate a trajectory because I want a different pose, craete a new one with the new pose
#  and the remaining distance to go

from numpy import array, linspace, zeros, identity, ceil, newaxis
from copy import deepcopy

# Define constants for updating the status of the trajectory
NOT_GENERATED: str = 'Not Generated, not started'
GENERATED: str = 'Generated, not started'
IN_PROGRESS: str = 'Generated, in-progress'
CLEARED: str = 'Generated, not finished'
COMPLETE: str = 'Generated, completed'

# Define constants for component type
NO_TYPE: str = ''
WAY_POINT: str = 'way-point'
SUB_TRAJECTORY: str = 'sub-trajectory'


class Trajectory:

    def __init__(self, distance_between_way_points: float, generate_trajectory_on_call: bool = False):

        # Define a variable to store the components that are included in the trajectory
        self.components_in_trajectory = []

        # Define a list to store the way-components that have not yet been reached in the trajectory
        self.components_remaining = []

        # Define a list to store the way-components that have been reached in the trajectory
        self.components_reached = []

        # Define the status of the trajectory
        self.status = NOT_GENERATED

        # Define the type of object stored in the component fields
        self.component_type = NO_TYPE

        # Define the distance between all points on the trajectory
        self.distance_between_way_points = distance_between_way_points

        # Generate the trajectory if called to by th constructor
        if generate_trajectory_on_call:
            self.generate()

    def generate(self) -> array:
        """
        Creates the individual components for the trajectory.

        Returns
        -------
        None
            The first way-point in the trajectory.
        """
        raise Exception("This function has not been implemented.")

    def clear(self) -> None:
        """
        Clears the current trajectory from the object and updates the status.

        Returns
        -------
        None
            The next way-point in the trajectory as None.
        """
        raise Exception("This function has not been implemented.")

    def update(self) -> array:
        """
        Updates the trajectory when the current way-point has been reached.

        Returns
        -------
        Array
            Returns the next point in the trajectory as an array. When the trajectory is complete,
            the next way-point is returned as None.
        """
        raise Exception("This function has not been implemented.")

    def get_current(self) -> array:
        """
        Returns the current point being reached in the trajectory.

        Returns
        -------
        Array
            Returns the next point in the trajectory as an array. If the trajectory has not been generated or is complete,
            the next way-point is returned as None.
        """
        raise Exception("This function has not been implemented.")

    def update_status(self):
        """
        Updates the status variable of the trajectory.
        """

        # If no components have been generated,
        if len(self.components_in_trajectory) == 0:
            self.status = NOT_GENERATED

        # If the components have been generated but no components have been reached,
        elif len(self.components_remaining) == len(self.components_in_trajectory):
            self.status = GENERATED

        # If there are still components remaining in the trajectory,
        elif len(self.components_remaining) > 0:
            self.status = IN_PROGRESS

        # If there are no components remaining in the trajectory and the number of components reached equals the number
        # of components in the trajectory,
        elif len(self.components_remaining) == 0 and len(self.components_in_trajectory) == len(self.components_reached):
            self.status = COMPLETE

        # Else raise an exception,
        else:
            raise Exception("Status could not be determined.\n"
                            "Number of " + self.component_type + "s in the trajectory is " +
                            str(len(self.components_in_trajectory)) + ".\n"
                            "Number of remaining " + self.component_type + "s in the trajectory is " +
                            str(len(self.components_remaining)) + ".\n"
                            "Number of " + self.component_type + "s reached in the trajectory is " +
                            str(len(self.components_reached)) + ".")

    def is_complete(self):
        """
        Checks if the trajectory is complete.
        """
        return self.status == COMPLETE


