"""Contains the constants used in the TrajectoryManagementNode logic"""

# Define constants for the possible actions
WAITING_FOR_ACTION: str = 'Waiting for action'
GENERATING_TRAJECTORY_OBJECTS: str = 'Generating New Trajectory Objects'
PREPARING_TO_NAVIGATE_CURRENT_TRAJECTORY: str = 'Preparing to Navigate Current Trajectory'
NAVIGATING_CURRENT_TRAJECTORY: str = 'Navigating Current Trajectory'
CLOSING_CURRENT_TRAJECTORY: str = 'Closing Current Trajectory'
INTERRUPT_CURRENT_TRAJECTORY: str = 'Interrupt Current Trajectory'

# Define constants for the possible types of scans
SINGLE_DIRECTION: str = 'Single Direction Scan'
BI_DIRECTION: str = 'Bi-directional Scan'
DUAL_LOBE_SCAN: str = 'Dual Lobe Scan'

# Define constants to declare which axis is being offset to make the trajectory
X_AXIS_OFFSET: int = 0
Y_AXIS_OFFSET: int = 1
Z_AXIS_OFFSET: int = 2

# Define constants for the types of interruptions that can occur
FULL_STOP: str = 'Full stop of all trajectories'
MOVE_TO_NEXT_TRAJECTORY_SEGMENT: str = 'Move to next trajectory segment'


# Define a constant to use for when there should be no waypoints
NO_WAYPOINTS: float = 1000