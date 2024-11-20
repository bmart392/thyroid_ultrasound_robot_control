"""Contains the code for the TrajectoryAction class"""

# Import standard python packages
from numpy import array, ndarray


# Define constants for the different types of trajectories
SURFACE_BASED: str = 'Surface Based'
POSE_TO_POSE: str = 'Pose to Pose'

# Define constants for the different locations of data
CURRENT_ROBOT_POSE: str = 'Current Robot Pose'
LAST_VALID_POSE: str = 'Last Valid Pose'


class TrajectoryAction:
    def __init__(self, trajectory_type: str, waypoint_spacing: float,
                 ending_offset: ndarray = None,
                 starting_pose: ndarray = None, starting_pose_src: str = None,
                 ending_pose: ndarray = None, ending_pose_src: str = None,
                 reset_image_mask_at_end: bool = False, include_standard_overrides: bool = False):
        self.trajectory_type: str = trajectory_type
        self.waypoint_spacing: float = waypoint_spacing
        self.ending_offset: array = ending_offset
        self.starting_pose: array = starting_pose
        self.starting_pose_src: str = starting_pose_src
        self.ending_pose: array = ending_pose
        self.ending_pose_src: str = ending_pose_src
        self.reset_image_mask_at_end: bool = reset_image_mask_at_end
        self.include_standard_overrides: bool = include_standard_overrides
