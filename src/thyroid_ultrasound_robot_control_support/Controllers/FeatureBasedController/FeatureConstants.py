"""
File containing the constants used in defining features.
"""

# Defining constants for denoting each axis
LIN_X: int = 0
LIN_Y: int = 1
LIN_Z: int = 2
ROLL_X: int = 3
PITCH_Y: int = 4
YAW_Z: int = 5

# Define constants for the status of each axis
LOCKED: bool = True
UNLOCKED: bool = False

# Define constants to select which frame is being referenced
FEATURE_FRAME: str = 'w.r.t. Frame of Feature Origin'
REFERENCE_FRAME: str = 'w.r.t. Frame of Reference Pose'

# Define the column containing the translation vector
TRANSLATION_COLUMN: int = int(3)

# Define the size of standard matrices
TRANSFORM_MATRIX: tuple = (4, 4)
ROW_VECTOR: tuple = (1, 3)
COLUMN_VECTOR: tuple = (3, 1)
AUGMENTED_COLUMN_VECTOR: tuple = (4, 1)


