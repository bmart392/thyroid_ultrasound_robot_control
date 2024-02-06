"""
Contains the code for the ComplexTrajectory class.
"""

# TODO - Low - Properly comment this file
# TODO - Low - Properly implement this class

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Trajectories.Trajectory import *


class ComplexTrajectory(Trajectory):

    def __init__(self, distance_between_way_points: float,
                 generate_trajectory_on_call: bool = False):

        super().__init__(distance_between_way_points=distance_between_way_points,
                         generate_trajectory_on_call=generate_trajectory_on_call)
