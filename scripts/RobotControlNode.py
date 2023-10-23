#!/usr/bin/env python3

"""
File containing the RobotControlNode class.
"""

# Import standard ros packages
from rospy import is_shutdown, init_node, Rate, Publisher, Subscriber
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
from franka_msgs.msg import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, Bool

# Import standard packages
from numpy import zeros, array, median, append, arange


# TODO - High - Verify position error is being properly received


class RobotControlNode:

    def __init__(self) -> None:

        # Define flag to know when the image is centered
        self.is_image_centered = False

        # Define arrays to store gain constants
        self.position_based_gains = array([.01, .01, .01, .01, .01, .01])
        self.force_based_gains = array([.01, .01, .005, .01, .01, .01])

        # Define variables to store data for force history
        self.force_history_length = 30
        self.force_history = [array([]), array([]), array([]), array([]), array([]), array([])]

        # Define control inputs to be used
        self.image_based_control_input = zeros(6)
        self.force_based_control_input = zeros(6)
        self.position_based_control_input = zeros(6)

        # Define variables to store relevant robot information
        self.robot_sensed_force = zeros(6)
        self.current_pose = zeros(6)

        # Define variables to store desired motion
        self.goal_pose = zeros(6)
        self.goal_force = zeros(6)

        # Define variables to store commands sent by the controller
        self.stop_motion_command = False
        self.use_image_feedback_flag = False
        self.use_pose_feedback_flag = False
        self.use_force_feedback_flag = False

        # Initialize the node
        init_node('robot_control_node')

        # Create control input subscribers
        Subscriber('/control_error/image_based', TwistStamped, self.image_based_control_error_callback)
        Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.robot_sensed_force_callback)
        Subscriber('/franka_state_controller/franka_states', FrankaState, self.robot_current_pose_callback)

        # Create goal state subscribers
        Subscriber('/goal/pose', PoseStamped, self.goal_pose_callback)
        Subscriber('/force_control/set_point', Float64, self.force_set_point_callback)

        # Create command subscribers
        Subscriber('/command/use_image_feedback', Bool, self.use_image_feedback_command_callback)
        Subscriber('/command/use_pose_feedback', Bool, self.use_pose_feedback_command_callback)
        Subscriber('/command/use_force_feedback', Bool, self.use_force_feedback_command_callback)

        # Create status publishers
        # self.was_last_goal_reached_status_publisher = Publisher('/status/goal_reached', Bool, queue_size=1)
        # self.current_pose_status_publisher = Publisher('/status/current_pose', PoseStamped, queue_size=1)

        # Create robot cartesian velocity publisher
        self.cartesian_velocity_publisher = Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

        # Define a publisher to publish if the image is centered
        # self.image_centered_publisher = Publisher('is_image_centered', Bool, queue_size=1)

    # Pull control inputs from message
    def image_based_control_error_callback(self, data: TwistStamped) -> None:

        # Define the acceptable limits of error for each dimension
        acceptable_errors = [0.005,  # meters in x
                             1.01,  # meters in y
                             1.01,  # meters in z
                             10,  # deg
                             10,  # deg
                             10,  # deg
                             ]

        # Define the control input gains for each dimension
        control_input_gains = [
            # [ k_p, k_d]
            [0.5, 0.0],  # x
            [1.0, 0.0],  # y
            [1.0, 0.0],  # z
            [1.0, 0.0],  # angle x
            [1.0, 0.0],  # angle y
            [1.0, 0.0],  # angle z
        ]

        # Pull the current error out of the twist
        current_error = zeros(6)
        current_error[0] = data.twist.linear.x
        current_error[1] = data.twist.linear.y
        current_error[2] = data.twist.linear.z
        current_error[3] = data.twist.angular.x
        current_error[4] = data.twist.angular.y
        current_error[5] = data.twist.angular.z

        # Define a temporary place to store the control inputs
        temp_control_inputs = array([])

        # Check if the error is within an acceptable amount
        for single_dimension_error, gains, single_dimension_acceptable_error in zip(current_error, control_input_gains,
                                                                                    acceptable_errors):

            # Calculate the control input for the given dimension
            temp_control_inputs = append(temp_control_inputs, current_error * gains[0])

        # Save the control input calculated here
        self.image_based_control_input = temp_control_inputs

    def robot_sensed_force_callback(self, data: WrenchStamped) -> None:
        """
        Saves the current force felt by the robot and calculates the median force felt by the robot in the recent past.

        Parameters
        ----------
        data
            The message containing the force felt by the robot.
        """

        # Pop out an element from each history if too many have been stored
        for dimension, index in zip(self.force_history, arange(6)):
            if len(dimension) >= self.force_history_length:
                self.force_history[index] = dimension[1:]

        # Add in the new value
        # self.force_history[0] = append(self.force_history[0], data.wrench.force.x)
        # self.force_history[1] = append(self.force_history[1], data.wrench.force.y)
        self.force_history[2] = append(self.force_history[2], data.wrench.force.z)
        # self.force_history[3] = append(self.force_history[3], data.wrench.torque.x)
        # self.force_history[4] = append(self.force_history[4], data.wrench.torque.y)
        # self.force_history[5] = append(self.force_history[5], data.wrench.torque.z)

        # Average the force history to find a more consistent force value
        self.robot_sensed_force[0] = 0  # mean(self.force_history[0]) 
        self.robot_sensed_force[1] = 0  # mean(self.force_history[1])
        self.robot_sensed_force[2] = median(self.force_history[2])
        self.robot_sensed_force[3] = 0  # mean(self.force_history[3])
        self.robot_sensed_force[4] = 0  # mean(self.force_history[4])
        self.robot_sensed_force[5] = 0  # mean(self.force_history[5])

    def robot_current_pose_callback(self, data: FrankaState) -> None:
        """
        Captures the current pose of the robot from messages sent by the robot controller.
        """
        self.current_pose[0] = data.O_T_EE[3]
        # self.current_pose[1] = data.O_T_EE[7]
        # self.current_pose[2] = data.O_T_EE[11]
        # self.calculate_goal_based_control_input()
        # self.was_last_goal_reached_status_publisher.publish(Bool(
        #     sqrt(sum((self.current_pose - self.goal_pose) ** 2)) <= .01  # m
        # ))
        # temp_current_pose = PoseStamped()
        # temp_current_pose.pose.position.x = self.current_pose[0]
        # temp_current_pose.pose.position.y = self.current_pose[1]
        # temp_current_pose.pose.position.z = self.current_pose[2]
        # self.current_pose_status_publisher.publish(temp_current_pose)

    def goal_pose_callback(self, data: PoseStamped):
        """
        Captures the desired pose sent to the node.
        """
        self.goal_pose[0] = data.pose.position.x
        # self.goal_pose[1] = data.pose.position.y
        # self.goal_pose[2] = data.pose.position.z
        # self.goal_pose[3] = data.twist.angular.x
        # self.goal_pose[4] = data.twist.angular.y
        # self.goal_pose[5] = data.twist.angular.z

    def force_set_point_callback(self, data: Float64) -> None:
        """
        Saves the force set-point to a local variable.

        Parameters
        ----------
        data
            The message containing the force set-point to save.
        """
        self.goal_force[2] = data.data

    def use_image_feedback_command_callback(self, data: Bool) -> None:
        """
        Update the use_image_feedback_flag based on the command value given.
        """
        if not self.use_image_feedback_flag == data.data:
            if data.data:
                print("Image feedback turned on.")
            else:
                print("Image feedback turned off.")
        self.use_image_feedback_flag = data.data

    def use_pose_feedback_command_callback(self, data: Bool) -> None:
        """
        Update the use_pose_feedback_flag based on the command value given.
        """
        if not self.use_pose_feedback_flag == data.data:
            if data.data:
                print("Moving to goal position.")
            else:
                print("Not moving to goal.")
        self.use_pose_feedback_flag = data.data

    def use_force_feedback_command_callback(self, data: Bool) -> None:
        """
        Update the use_force_feedback_flag based on the command value given.
        """
        if not self.use_force_feedback_flag == data.data:
            if data.data:
                print("Force feedback turned on.")
            else:
                print("Force feedback turned off.")
        self.use_force_feedback_flag = data.data

    def main(self) -> None:
        """
        Calculates the correct control input based on the current error in the system and
        which control modes are active.
        """

        # Create blank array to store the final control input as an array
        control_input_array = zeros(6)

        if self.use_pose_feedback_flag:
            control_input_array = control_input_array + (self.position_based_gains *
                                                         (-self.current_pose + self.goal_pose))
            # print("Goal based control input used.")

        if self.use_image_feedback_flag:
            control_input_array = control_input_array + self.image_based_control_input
            # print("Image centering control input used.")

        if self.use_force_feedback_flag:
            control_input_array = control_input_array + (self.force_based_gains *
                                                         (-self.goal_force + self.robot_sensed_force))
            # print("Force feedback control input: ", node.force_based_control_input[2])

        # Create a message and fill it with the desired control input
        control_input_message = TwistStamped()
        control_input_message.twist.linear.x = control_input_array[0]
        control_input_message.twist.linear.y = control_input_array[1]
        control_input_message.twist.linear.z = control_input_array[2]
        control_input_message.twist.angular.x = control_input_array[3]
        control_input_message.twist.angular.y = control_input_array[4]
        control_input_message.twist.angular.z = control_input_array[5]

        self.cartesian_velocity_publisher.publish(control_input_message)


if __name__ == '__main__':

    # Create the node
    node = RobotControlNode()

    # Set publishing rate
    publishing_rate = Rate(100)  # hz

    print("Node initialized. Press ctrl+c to terminate.")

    while not is_shutdown():
        # Run the main function of the node
        node.main()

        # Wait to run the node again
        publishing_rate.sleep()
