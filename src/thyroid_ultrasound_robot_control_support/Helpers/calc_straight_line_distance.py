def calc_straight_line_distance(point_1: tuple, point_2: tuple) -> float:
    """
    Calculates the straight-line distance between two 3D points.

    Parameters
    ----------
    point_1 :
        A tuple containing 3 numeric values representing a 3D point.
    point_2 :
        A tuple containing 3 numeric values representing a 3D point.

    Returns
    -------
    float
        The straight-line distance between point 1 and 2.
    """
    if len(point_1) != 3 and len(point_2) != 3:
        raise Exception("Either the size of point 1," + str(len(point_1)) +
                        ", and point 2," + str(len(point_2)) +
                        " is not 3.")

    return ((point_2[0] - point_1[0]) ** 2 +
            (point_2[1] - point_1[1]) ** 2 +
            (point_2[2] - point_1[2]) ** 2) ** 0.5
