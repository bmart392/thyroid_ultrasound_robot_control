#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
from franka_msgs.msg import FrankaState
from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool

from numpy import sign, zeros, array, sum, sqrt, mean

class RobotControlNode:

    def __init__(self) -> None:

        # Define arrays to store gain constants
        self.position_based_gains = array([.01, .01, .01, .01, .01, .01])
        self.force_based_gains = array([.01, .01, .001, .01, .01, .01])

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
        self.center_image_command = False
        self.move_to_goal_command = False
        self.use_force_feedback_command = False
        
        # Initialize the node
        rospy.init_node('robot_control_node')

        # Set publishing rate
        self.rate = rospy.Rate(100)  # hz
        
        # Create control input subscribers
        self.image_based_control_input_subscriber = rospy.Subscriber('/control_input/image_based', TwistStamped, self.image_based_control_input_callback)
        self.robot_sensed_force_subscriber = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.robot_sensed_force_callback)
        self.robot_current_pose_subscriber = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.robot_current_pose_callback)

        # Create goal state subscribers
        self.goal_pose_subscriber = rospy.Subscriber('/goal/pose', PoseStamped, self.goal_pose_callback)
        self.goal_force_subscriber = rospy.Subscriber('/goal/force', WrenchStamped, self.goal_force_callback)
        
        # Create command subscirbers
        self.stop_motion_subscriber = rospy.Subscriber('/command/stop_motion', Bool, self.stop_motion_callback)
        self.center_image_subscriber = rospy.Subscriber('/command/center_image', Bool, self.center_image_callback)
        self.move_to_goal_subscriber = rospy.Subscriber('/command/move_to_goal', Bool, self.move_to_goal_callback)
        self.use_force_feedback_subscriber = rospy.Subscriber('/command/use_force_feedback', Bool, self.use_force_feedback_callback)

        # Create status publishers
        self.was_last_goal_reached_status_publisher = rospy.Publisher('/status/goal_reached', Bool, queue_size=1)
        self.current_pose_status_publisher = rospy.Publisher('/status/current_pose', PoseStamped, queue_size=1)

        # Create robot cartesian velocity publisher
        self.cartesian_velocity_publisher = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

    # Pull control inputs from message
    def image_based_control_input_callback(self, data: TwistStamped):

        self.image_based_control_input[0] = data.twist.linear.x 
        self.image_based_control_input[1] = data.twist.linear.y
        self.image_based_control_input[2] = data.twist.linear.z
        self.image_based_control_input[3] = data.twist.angular.x
        self.image_based_control_input[4] = data.twist.angular.y
        self.image_based_control_input[5] = data.twist.angular.z

    # Pull force values felt currently by robot
    def robot_sensed_force_callback(self, data: WrenchStamped):

        # Pop out an element from each history if too many have been stored
        for dimension in self.force_history:
            if len(dimension) > self.force_history_length:
                dimension.pop(0)

        # Add in the the new value
        # self.force_history[0].append(data.wrench.force.x)
        # self.force_history[1].append(data.wrench.force.y)
        self.force_history[2].append(data.wrench.force.z)
        # self.force_history[3].append(data.wrench.torque.x)
        # self.force_history[4].append(data.wrench.torque.y)
        # self.force_history[5].append(data.wrench.torque.z)

        # Average the force history to find a more real force value
        self.robot_sensed_force[0] = 0  # mean(self.force_history[0]) 
        self.robot_sensed_force[1] = 0  # mean(self.force_history[1])
        self.robot_sensed_force[2] = mean(self.force_history[2])
        self.robot_sensed_force[3] = 0  # mean(self.force_history[3])
        self.robot_sensed_force[4] = 0  # mean(self.force_history[4])
        self.robot_sensed_force[5] = 0  # mean(self.force_history[5])

    # Pull current pose of the robot
    def robot_current_pose_callback(self, data: FrankaState):
        self.current_pose[0] = data.O_T_EE[3]
        self.current_pose[1] = data.O_T_EE[7]
        self.current_pose[2] = data.O_T_EE[11]
        self.calculate_goal_based_control_input()
        self.was_last_goal_reached_status_publisher.publish(Bool(
            sqrt(sum((self.current_pose - self.goal_pose) ** 2)) <= .01  # m
        ))
        temp_current_pose = PoseStamped()
        temp_current_pose.pose.position.x = self.current_pose[0]
        temp_current_pose.pose.position.y = self.current_pose[1]
        temp_current_pose.pose.position.z = self.current_pose[2]
        self.current_pose_status_publisher.publish(temp_current_pose)

    # Pull goal pose from message
    def goal_pose_callback(self, data: PoseStamped):
        self.goal_pose[0] = data.pose.position.x 
        self.goal_pose[1] = data.pose.position.y
        self.goal_pose[2] = data.pose.position.z
        # self.goal_pose[3] = data.twist.angular.x
        # self.goal_pose[4] = data.twist.angular.y
        # self.goal_pose[5] = data.twist.angular.z

    # Pull desired force from message
    def goal_force_callback(self, data: WrenchStamped):
        self.goal_force[0] = data.wrench.force.x 
        self.goal_force[1] = data.wrench.force.y
        self.goal_force[2] = data.wrench.force.z
        self.goal_force[3] = data.wrench.torque.x
        self.goal_force[4] = data.wrench.torque.y
        self.goal_force[5] = data.wrench.torque.z

        self.calculate_force_based_control_input()

    # Pull stop motion command from message
    def stop_motion_callback(self, data: Bool):
        self.stop_motion_command = data.data

    # Pull center image command from message
    def center_image_callback(self, data: Bool):
        self.center_image_command = data.data

    # Pull move to goal command from message
    def move_to_goal_callback(self, data: Bool):
        self.move_to_goal_command = data.data

    # Pull use force feedback command from message
    def use_force_feedback_callback(self, data: Bool):
        self.use_force_feedback_command = data.data

    # Calculate the control input based on the goal pose
    def calculate_goal_based_control_input(self):
        self.position_based_control_input = self.position_based_gains * (self.current_pose + self.goal_pose)

    # Calculate the control input based on the currently sensed force
    def calculate_force_based_control_input(self):
        self.force_based_control_input = self.force_based_gains * (-self.goal_force + self.robot_sensed_force)



if __name__ == '__main__':
    
    node = RobotControlNode()
    print("Node initialized. Press ctrl+c to terminate.")

    while not rospy.is_shutdown():

        # Create blank array to store the final control input as an array
        control_input_array = zeros(6)

        if node.move_to_goal_command:
            control_input_array = control_input_array + node.position_based_control_input
            print("Goal based control input used.")

        if node.center_image_command:
            control_input_array = control_input_array + node.image_based_control_input
            print("Image centering control input used.")

        if node.use_force_feedback_command:
            control_input_array = control_input_array + node.force_based_control_input 
            print("Force feedback control input used.")

        # Create a message and fill it with the desired control input
        control_input_message = TwistStamped()
        control_input_message.twist.linear.x = control_input_array[0]
        control_input_message.twist.linear.y = control_input_array[1]
        control_input_message.twist.linear.z = control_input_array[2]
        control_input_message.twist.angular.x = control_input_array[3]
        control_input_message.twist.angular.y = control_input_array[4]
        control_input_message.twist.angular.z = control_input_array[5]

        node.cartesian_velocity_publisher.publish(control_input_message)
        
        # node.was_last_goal_reached_status_publisher.publish(Bool(node.))
        
        node.rate.sleep()