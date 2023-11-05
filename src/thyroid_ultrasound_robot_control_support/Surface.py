#!/usr/bin/env python3

"""
File containing the Surface class.
"""

# Import standard python packages
from numpy import array, cross, dot, sqrt


class Surface:

    def __init__(self, defining_points: array):
        """
        Create a new surface using the two vector approach.

        Parameters
        ----------
        defining_points
            An array of 3 points representing the origin point of the surface and two vectors that lie on the surface
        """

        # Define the point on the plane as the first point given
        self.point_on_plane = defining_points[0]

        # Calculate the vector between the origin point and the second point
        vector_1 = defining_points[1] - defining_points[0]

        # Calculate the vector between the origin point and the second point
        vector_2 = defining_points[2] - defining_points[0]

        # Calculate the normal vector of the surface
        normal_vector = cross(vector_1, vector_2)

        # Transform the normal vector to a unit normal vector
        self.unit_normal_vector = normal_vector / self.vector_magnitude(normal_vector)

    def distance_to_surface(self, point_to_check: array):
        """
        Calculate the distance to the surface from a given point.

        Parameters
        ----------
        point_to_check
            An array representing an x, y, z coordinate that is some distance away from the surface.
        """

        # Calculate the distance to the surface using the dot product
        return dot(self.unit_normal_vector, point_to_check - self.point_on_plane)

    @staticmethod
    def vector_magnitude(vector: array):
        """
        Calculate the magnitude of the given vector.
        """
        return sqrt(sum(vector ** 2))


