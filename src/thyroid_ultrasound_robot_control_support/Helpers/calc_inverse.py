from numpy import array, transpose, ndarray, identity, newaxis


def calc_inverse(input_transformation_matrix) -> ndarray:
    """
    Calculates the inverse of a homogeneous transformation matrix.

    Parameters
    ----------
    input_transformation_matrix
        A 4x4 homogeneous transformation array. Can be any type as long as it can be converted to a numpy array.
    Returns
    -------
    ndarray
        A 4x4 numpy array containing the inverse of the given array.

    """

    # Convert the input to a numpy array
    if type(input_transformation_matrix) != ndarray:
        input_transformation_matrix = array(input_transformation_matrix)

    # Ensure that the array has the correct shape,
    if input_transformation_matrix.shape != (4, 4):
        raise Exception("The matrix of shape " + str(input_transformation_matrix.shape) +
                        " did not have the expected shape of (4, 4).")

    # Define a matrix to store the result
    inverse_matrix = identity(4)

    # Calculate the inverse of the rotation matrix and store it
    inverse_rotation = transpose(input_transformation_matrix[0:3, 0:3])
    inverse_matrix[0:3, 0:3] = inverse_rotation

    # Calculate the inverse of the translation matrix and store it
    inverse_translation = -inverse_rotation @ input_transformation_matrix[0:3, 3].reshape((3, 1))
    inverse_matrix[0:3, 3, newaxis] = inverse_translation

    return inverse_matrix


if __name__ == '__main__':
    t_0_1 = array([
        [0, -1, 0, -1],
        [1, 0, 0, -1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    t_1_0 = calc_inverse(t_0_1)
    print(t_0_1)
    print(t_1_0)
    p_1 = array([1, 1, 0, 1]).reshape((4, 1))
    print(p_1)
    p_0 = t_1_0 @ p_1
    print(p_0)
    print(t_0_1 @ p_0)
