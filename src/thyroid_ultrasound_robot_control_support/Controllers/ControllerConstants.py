#!/usr/bin/env python3

"""
File containing constants used for managing the PID controllers.
"""

# Define constants for each controller and controller channel
P_GAIN: int = int(0)
I_GAIN: int = int(1)
D_GAIN: int = int(2)

# Define a tag for each sub-controller in the FeatureController object
X_LINEAR_CONTROLLER: int = int(0)
Y_LINEAR_CONTROLLER: int = int(1)
Z_LINEAR_CONTROLLER: int = int(2)
X_ANGULAR_CONTROLLER: int = int(3)
Y_ANGULAR_CONTROLLER: int = int(4)
Z_ANGULAR_CONTROLLER: int = int(5)

# Define a tuple of all the sub-controller tags in the FeatureController object
CONTROLLER_KEYS = (X_LINEAR_CONTROLLER, Y_LINEAR_CONTROLLER, Z_LINEAR_CONTROLLER,
                   X_ANGULAR_CONTROLLER, Y_ANGULAR_CONTROLLER, Z_ANGULAR_CONTROLLER)

# Define additional tags to use in the RobotControlNode
COMBINED_POSITION_CONTROLLER: int = 0
