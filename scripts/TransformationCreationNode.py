#!/usr/bin/env python3

"""
File for creating the transformation between the base frame and the end effector.
"""

# Import standard ROS packages
from std_msgs.msg import MultiArrayDimension
from franka_msgs.msg import FrankaState

# Import standard python packages
from numpy import array, pi, identity, degrees
from numpy import cos as c
from numpy import sin as s

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import Float64MultiArrayStamped

# Import custom python packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *


class TransformationCreationNode(BasicNode):
    """
    Creates a node for publishing the homogeneous transformation matrix from the base frame to the end
    effector of the robot based on the joint angles of the robot.
    """

    def __init__(self):

        # Add call to super class init
        super().__init__()

        # Initialize the node
        init_node('TransformationCreationNode')

        # Create a subscriber to listen for the joint angles
        Subscriber(ROBOT_STATE, FrankaState, self.calculate_ee_transformation)

        # Create a publisher to publish the transformation from the base frame to the end effector frame
        self.transformation_publisher = Publisher(ROBOT_DERIVED_POSE, Float64MultiArrayStamped, queue_size=1)

        # Define the transformation from the flange frame to the custom end effector
        self.flange_to_ee = array([
            array([1, 0, 0, 0]),
            array([0, 1, 0, 0]),
            array([0, 0, 1, .23]),
            array([0, 0, 0, 1]),
        ])

        # Define a variable to store the DH parameters of the robot
        # Order of parameters is : a, d , alpha, theta
        self.dh_params = [
            (0, 0.333, 0, None),
            (0, 0, -pi/2, None),
            (0, 0.316, pi/2, None),
            (0.0825, 0, pi/2, None),
            (-0.0825, 0.384, -pi/2, None),
            (0, 0, pi/2, None),
            (0.088, 0, pi/2, None),
            (0, 0.107, 0, 0),
        ]

    def calculate_ee_transformation(self, data: FrankaState):
        """
        Calculate the 4x4 homogeneous transformation from the base frame to the end effector frame.

        Parameters
        ----------
        data
            The FrankaState message containing the joint angles of the robot.
        """
        # Pull the joint angles out of the FrankaState message
        joint_angles = list(data.q)

        # Add a None object on to the list to represent the flange
        joint_angles.append(None)

        # Create a matrix to store the result of the iterative calculation process
        result_array = identity(4)

        # Iteratively calculate the transformation to the end effector
        for joint_angle, ii in zip(joint_angles, range(len(joint_angles))):
            result_array = result_array @ self.create_homogeneous_transformation_matrix_from_dh(ii, theta=joint_angle)

        # Add the end effector transformation to the result
        result_array = result_array @ self.flange_to_ee

        # Populate the transformation message
        transformation_msg = Float64MultiArrayStamped()
        transformation_msg.header.stamp = data.header.stamp
        transformation_msg.data.layout.data_offset = 0
        transformation_msg.data.layout.dim.append(MultiArrayDimension)
        transformation_msg.data.layout.dim[0].label = "row"
        transformation_msg.data.layout.dim[0].size = 4
        transformation_msg.data.layout.dim[0].stride = 16
        transformation_msg.data.layout.dim.append(MultiArrayDimension)
        transformation_msg.data.layout.dim[1].label = "column"
        transformation_msg.data.layout.dim[1].size = 4
        transformation_msg.data.layout.dim[1].stride = 0
        transformation_msg.data.data = result_array.reshape(16)

        # Publish the transformation message
        self.transformation_publisher.publish(transformation_msg)

    def create_homogeneous_transformation_matrix_from_dh(self,
                                                         joint_num: int,
                                                         a: float = None,
                                                         d: float = None,
                                                         alpha: float = None,
                                                         theta: float = None,) -> array:
        """
        Calculates a homogeneous transformation matrix given some number of DH parameters.

        This function uses the modified DH parameters laid out in Introduction to Robotics: Mechanics and Control
        (3rd Edition).

        Parameters
        ----------
        joint_num:
            The number of the joint parameter used to populate the DH parameters not given as arguments.
        a:
            The DH parameter a.
        d:
            The DH parameter d.
        alpha:
            The DH parameter alpha.
        theta:
            The DH parameter theta.
        """

        # Replace any parameters not given with those that have been provided
        if a is None:
            a = self.dh_params[joint_num][0]
        if d is None:
            d = self.dh_params[joint_num][1]
        if alpha is None:
            alpha = self.dh_params[joint_num][2]
        if theta is None:
            theta = self.dh_params[joint_num][3]

        # Ensure that all dh parameters are defined prior to calculating the transformation
        error_msg = None
        if a is None:
            error_msg = "a"
        if d is None:
            error_msg = "d"
        if alpha is None:
            error_msg = "alpha"
        if theta is None:
            error_msg = "theta"
        if error_msg is not None:
            raise Exception("The " + error_msg + " parameter was not provided.")

        # Calculate the transformation matrix
        return array([
            array([c(theta), -s(theta), 0, a]),
            array([s(theta)*c(alpha), c(theta)*c(alpha), -s(alpha), -d*s(alpha)]),
            array([s(theta)*s(alpha), c(theta)*s(alpha), c(alpha), d*c(alpha)]),
            array([0, 0, 0, 1]),
        ])


if __name__ == "__main__":

    # Initialize the node
    node = TransformationCreationNode()

    # Notify the user that the node has been initialized
    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Spin while callbacks handle node function
    spin()
