"""
File containing the constants used in defining features.
"""

# Define constants for noting which translation axes are locked
X_LOCKED: str = 'X locked'
Y_LOCKED: str = 'Y locked'
Z_LOCKED: str = 'Z locked'
X_UNLOCKED: str = 'X unlocked'
Y_UNLOCKED: str = 'Y unlocked'
Z_UNLOCKED: str = 'Z unlocked'

# Define constants for noting which rotation axes are locked
ROLL_LOCKED: str = 'Roll locked'
PITCH_LOCKED: str = 'Pitch locked'
YAW_LOCKED: str = 'Yaw locked'
ROLL_UNLOCKED: str = 'Roll unlocked'
PITCH_UNLOCKED: str = 'Pitch unlocked'
YAW_UNLOCKED: str = 'Yaw unlocked'

# Define constant indexes for each axis
X_AXIS: int = int(0)
Y_AXIS: int = int(1)
Z_AXIS: int = int(2)

# Define the column containing the translation vector
TRANSLATION_COLUMN: int = int(3)

# Define the size of standard matrices
TRANSFORM_MATRIX: tuple = (4, 4)
ROW_VECTOR: tuple = (1, 3)
COLUMN_VECTOR: tuple = (3, 1)
AUGMENTED_COLUMN_VECTOR: tuple = (4, 1)

# Define the keys used to store the results of the error function
TRANSLATION_ERROR_WRT_FEATURE: str = 'Translation error with respect to the feature frame'
TRANSLATION_ERROR_WRT_ORIGIN: str = 'Translation error with respect to the origin frame'
ROTATIONAL_ERROR: str = 'Rotational error'


