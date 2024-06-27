#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String
from numpy import sign


min_control_speed = 0.001 # m/s
max_control_speed = 0.025 # m/s

def centroid_error_callback(data):
    global current_error
    current_error = data.data

def arm_state_callback(message):
    global cartesian_position
    global cartesian_orientation
    global joint_torques
    cartesian_position = message.ee_pose.pose.position
    cartesian_orientation = message.ee_pose.pose.orientation
    joint_torques = message.joint_torques

def calculate_control_input(error):
    control_input = 0.

    if abs(error) >= .001:
        global min_control_speed
        global max_control_speed
        control_input = .1 * error
        if abs(control_input) > max_control_speed:
            control_input = max_control_speed * sign(control_input)
        if abs(control_input) < min_control_speed:
            control_input = min_control_speed * sign(control_input)
    return  control_input

if __name__ == '__main__':
    # initialise ros node
    rospy.init_node('error_based_motion')

    # initialize current_error value
    current_error = 0.0

    # initialize values for currrent position and orientation
    cartesian_position = None
    cartesian_orientation = None

    # Create the publisher to publish the desired joint velocities
    publisher = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

    # Create a subscriber to listen to the current state of the robot
    robot_state_subscriber = rospy.Subscriber('/arm/state', ManipulatorState, arm_state_callback)

    # Create a subscriber to listen to the error gathered from ultrasound images
    centroid_error_subscriber = rospy.Subscriber('/image_data/centroid_error', Float64, centroid_error_callback)
    """ 
    # Wait for the robot to pusblish data about its current state before continuing
    while not rospy.is_shutdown() and cartesian_position is None and cartesian_orientation is None:
        print("Waiting for joint state.")
        rospy.sleep(.2)

    # Save starting position of the robot
    initial_position = cartesian_position

    # Set the destnation goal of the position
    goal_position = initial_position + [0, 0, .25]
    """
    # Set rate for publishing new velocities
    rate = rospy.Rate(100) #hz
    
    while not rospy.is_shutdown():
        
        # Create a velocity message that will instruct the robot to
        # move in the z-axis of the base frame.
        velocity = TwistStamped()
        velocity.twist.linear.z = calculate_control_input(current_error)

        # Publish the velocity message to the Panda driver at a
        # frequency of 100Hz
        publisher.publish(velocity)
        
        # Wait to publish the next message
        rate.sleep()
        
    # Publish an empty TwistStamped to ensure that the arm stops moving
    publisher.publish(TwistStamped())