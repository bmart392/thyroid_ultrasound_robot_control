"""
Contains the code for the vector_magnitude function.
"""

# Import standard python packages
from numpy import array, sqrt


def vector_magnitude(vector: array):
    """
    Calculate the magnitude of the given vector.
    """
    # Calculate the vector magnitude
    if len(vector.shape) == 1 and vector.shape[0] == 3:
        temp_sum = sum(vector ** 2)
    elif len(vector.shape) == 2 and vector.shape == (3, 1):
        temp_sum = sum(sum(vector ** 2))
    else:
        raise Exception("The array passed into this function was formatted incorrectly.\n" 
                        "The shape of the array given was: " + str(vector.shape))
    return sqrt(temp_sum)
