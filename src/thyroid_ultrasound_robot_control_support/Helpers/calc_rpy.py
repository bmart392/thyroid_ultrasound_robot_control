from numpy import array, arctan2, ndarray, identity, rad2deg


def calc_rpy(rotation_matrix: ndarray) -> tuple:
    """
    Calculates the equivalent roll, pitch, and yaw angles for the given rotation matrix.
    Parameters
    ----------
    rotation_matrix
        A (3, 3) numpy array containing a valid rotation matrix

    Returns
    -------
    tuple
        A tuple containing, in this order, roll, pitch, and yaw in radians.
    """
    # Ensure that the input is an array
    if type(rotation_matrix) != ndarray:
        rotation_matrix = array(rotation_matrix)

    # Ensure that the input array has the correct shape
    if rotation_matrix.shape != (3, 3):
        raise Exception("The shape of the given rotation matrix, " + str(rotation_matrix.shape) +
                        ", was not (3, 3).")

    # Ensure that the values in the rotation matrix are valid
    if rotation_matrix[0, 0] == 0 and rotation_matrix[2, 2] == 0:
        raise Exception("The rotation matrix cannot have values of 0 in the top left and bottom right corners.")

    # Calculate roll, pitch, and yaw
    return (rad2deg(arctan2(rotation_matrix[2, 1], arctan2(2, 2))),
            rad2deg(arctan2(-rotation_matrix[2, 0], (rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2) ** 0.5)),
            rad2deg(arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])),
            )


if __name__ == '__main__':
    print(calc_rpy(
        array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]])
    )
    )
