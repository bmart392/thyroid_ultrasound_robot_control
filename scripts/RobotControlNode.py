#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
from franka_msgs import FrankaState
from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool

from numpy import sign, zeros

class RobotControlNode:

    def __init__(self) -> None:
        
        # Define control inputs to be used
        self.image_based_control_input = zeros(6)
        self.force_based_control_input = zeros(6)
        self.position_based_control_input = zeros(6)
        
        # Define variables to store relevant robot information
        self.force_sensed_force = zeros(6)
        self.current_pose = zeros(6)

        # Define variables to store desired motion
        self.goal_pose = zeros(6)
        self.desired_force = zeros(6)
        
        # Define variables to store commands sent by the controller
        self.stop_motion_command = False
        self.center_image_command = False
        self.move_to_goal_command = False
        self.use_force_feedback_command = False
        
        # Initialize the node
        rospy.init_node('robot_control_node')
        
        # Create control input subscribers
        self.image_based_control_input_subscriber = rospy.Subscriber('/control_input/image_based', TwistStamped, self.image_based_control_input_callback)
        self.robot_sensed_force_subscriber = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.robot_sensed_force_callback)
        self.robot_current_pose_subscriber = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.robot_current_pose_callback)

        # Create command subscribers
        self.goal_pose_subscriber = rospy.Subscriber('/command/goal_pose', TwistStamped, self.goal_pose_callback)
        self.desired_force_subscriber = rospy.Subscriber('/command/desired_force', WrenchStamped, self.desired_force_callback)
        self.stop_motion_subscriber = rospy.Subscriber('/command/stop_motion', Bool, self.stop_motion_callback)

        # Create status publishers
        self.was_last_goal_reached_status_publisher = rospy.Publisher('/status/goal_reached', Bool, queue_size=1)

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
        self.force_based_control_input[0] = data.wrench.force.x 
        self.force_based_control_input[1] = data.wrench.force.y
        self.force_based_control_input[2] = data.wrench.force.z
        self.force_based_control_input[3] = data.wrench.torque.x
        self.force_based_control_input[4] = data.wrench.torque.y
        self.force_based_control_input[5] = data.wrench.torque.z

    # Pull current pose of the robot
    def robot_current_pose_callback(self, data: FrankaState):
        self.current_pose[0] = data.O_T_EE[3]
        self.current_pose[1] = data.O_T_EE[7]
        self.current_pose[2] = data.O_T_EE[11]

    # Pull goal pose from message
    def goal_pose_callback(self, data: TwistStamped):
        self.goal_pose[0] = data.twist.linear.x 
        self.goal_pose[1] = data.twist.linear.y
        self.goal_pose[2] = data.twist.linear.z
        self.goal_pose[3] = data.twist.angular.x
        self.goal_pose[4] = data.twist.angular.y
        self.goal_pose[5] = data.twist.angular.z

    # Pull desired force from message
    def desired_pose_callback(self, data: WrenchStamped):
        self.desired_force[0] = data.wrench.force.x 
        self.desired_force[1] = data.wrench.force.y
        self.desired_force[2] = data.wrench.force.z
        self.desired_force[3] = data.wrench.torque.x
        self.desired_force[4] = data.wrench.torque.y
        self.desired_force[5] = data.wrench.torque.z

    # Pull stop motion command from message
    def stop_motion_callback(self, data: Bool):
        self.stop_motion_command = data.data