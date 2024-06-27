#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
from franka_msgs import FrankaState
from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool
from numpy import sign, array, zeros, sqrt, sum
from motion_constants import *
from motion_helpers import *

class motion_controller:
    def __init__(self) -> None:
        self.cartesian_position_history = []
        self.image_centroid_error_history = []
        self.external_force_history = []
        self.thyroid_in_image_status = False
        self.desired_end_effector_force = 0.1  # N
        self.allowable_centroid_error = .1
        self.acceptable_cartesian_error = .1
        self.standard_scan_step = array([0.01, 0.0, 0.0])
        self.move_goal = None
        self.velocity_publisher = None
        self.centroid_error_subscriber = None
        self.thyroid_in_image_subscriber = None
        self.robot_state_subscriber = None
        self.force_readings_subscriber = None

        # initialize ros node
        rospy.init_node('motion_controller')
        
        # create publishers and subscribers
        self.init_publishers_and_subscirbers()

    #--------------------------------------------
    # Callback Functions
    #--------------------------------------------

    # capture the cartesian position of the robot
    def cartesian_position_callback(data: FrankaState, self):
                
        # limit the number of previous values stored to 5
        if len(self.cartesian_position_history) >= 5:
            self.cartesian_position_history.pop()

        # add the new value to the list
        self.cartesian_position_history.insert(0,
            (data.header.stamp.sec,
            array([data.O_T_EE[3],
            data.O_T_EE[7],
            data.O_T_EE[11]]))
            )

    # capture the error of the position of image centroid
    def centroid_error_callback(data: Float64, self):

        # remove oldest error value if more 5 have already been saved
        if len(self.image_centroid_error_history) >= 5:
            self.image_centroid_error_history.pop()
        
        # save the new error value
        self.image_centroid_error_history.append((0, array([0, data.data, 0])))

    # capture the current force felt by the robot
    def force_value_callback(data: WrenchStamped, self):

        # remove the oldest force value if more than 5 have already been saved
        if len(self.external_force_history) >= 5:
            self.external_force_history.pop()
        
        # save the new error value
        self.external_force_history.insert((data.head.time.sec, array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])))

    # capture if the thyroid is in the current image
    def thyroid_in_image_callback(data: Bool, self):
        self.thyroid_in_image_status = data.data
    
    #--------------------------------------------
    # Control Input Calculation Functions
    #--------------------------------------------

    # calculate the control input based on the image error
    def image_error_calculate_control_input(self):
        k_p = .1   
        min_control_speed = 0.001 # m/s
        max_control_speed = 0.025 # m/s
        error = self.image_centroid_error_history[0]
        return pd_controller(k_p, 0, error, 0, min_control_speed, max_control_speed)

    # calculate the control input based on the force error
    def force_error_calculate_control_input(self):
        k_p = .1
        k_d = 0
        min_control_speed = .001  # m/s
        max_control_speed = .025  # m/s
        error, error_dot = calculate_error(self.desired_end_effector_force, self.external_force_history)
        return pd_controller(k_p, k_d, error, error_dot, min_control_speed, max_control_speed)

    # calculate the control input based on the cartesian position error
    def cartesian_position_calculate_control_input(self):
        k_p = .1
        k_d = 0.
        min_control_speed = .001  # m/s
        max_control_speed = .025  # m/s
        error, error_dot  = calculate_error(self.move_goal, self.cartesian_position_history)
        return pd_controller(k_p, k_d, error, error_dot, min_control_speed, max_control_speed)

    #--------------------------------------------
    # Define ROS features
    #--------------------------------------------
    
    # create publisher and subscriber objects
    def init_publishers_and_subscirbers(self):
        # Create the publisher to publish the desired joint velocities
        self.velocity_publisher = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

        # Create a subscriber to listen to the error gathered from ultrasound images
        self.centroid_error_subscriber = rospy.Subscriber('/image_data/centroid_error', Float64, self.centroid_error_callback)

        # Create a subscriber to listen to the error gathered from ultrasound images
        self.thyroid_in_image_subscriber = rospy.Subscriber('/image_data/thyroid_in_image', Bool, self.thyroid_in_image_callback)

        # Create a subscriber to listen to the robot state
        self.robot_state_subscriber = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.cartesian_position_callback)

        # Create a subscriber to listen to the force readings from the robot
        self.force_readings_subscriber = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.force_value_callback)

    #--------------------------------------------
    # Helper functions
    #--------------------------------------------
    

if __name__ == '__main__':

    # create motion_controller object and start up ROS objects
    controller = motion_controller()

    #--------------------------------------------
    # Define state machine parameters
    #--------------------------------------------

    # initialize status of procedure
    procedure_complete_flag = False

    # initialize the state of the procedure
    procedure_state = CHECK_SETUP  # beginning of procedure
    previous_procedure_state = None

    # initialize flag indicating if in waypoint finding or scanning portion of procedure
    current_objective = WAYPOINT_FINDING

    # intialize direction of scanning
    current_direction = DIRECTION_TORSO

    #--------------------------------------------
    # Define variables to store results from procedure
    #--------------------------------------------

    # initialize empty stored sate for start of procedure
    procedure_origin = array([])

    # initialize empty array to store the path waypoints
    procedure_waypoints = []

    # Set rate for publishing new velocities
    rate = rospy.Rate(100) #hz

    # loop until the routine has been finished or interrupted
    while not rospy.is_shutdown() and not procedure_complete_flag:

        if procedure_state == CHECK_SETUP:

            # check that the thyroid is in the image
            if not controller.thyroid_in_image_status:
                rospy.shutdown()

            # change states to center the thyroid in the image
            previous_procedure_state = procedure_state
            procedure_state = CENTER_IMAGE
            
        if procedure_state == CENTER_IMAGE:

            # move the robot to center the centroid within the allowable error
            if abs(controller.image_centroid_error_history[0]) > controller.allowable_centroid_error:

                # calculate required control input
                centroid_control_inputs = controller.image_error_calculate_control_input()

                # calculate force control inputs
                force_control_inputs = controller.force_error_calculate_control_input()

                # generate a message to use to send control inputs
                centroid_correction_velocity = TwistStamped()

                # assign values to the message
                centroid_correction_velocity.twist.linear.x = centroid_control_inputs[0] + force_control_inputs[0]
                centroid_correction_velocity.twist.linear.y = centroid_control_inputs[1] + force_control_inputs[1]
                centroid_correction_velocity.twist.linear.z = centroid_control_inputs[2] + force_control_inputs[2]

                # publish the message
                controller.velocity_publisher.publish(centroid_correction_velocity)

            else:
                
                # if the list of procedure_waypoints list is empty, save the current position as the origin
                if len(procedure_waypoints) == 0:
                    procedure_origin = controller.cartesian_position_history[0][1]

                # set placement index based on direction of movement
                if current_direction == DIRECTION_HEAD:
                    placement_index = -1
                elif current_direction == DIRECTION_TORSO:
                    placement_index == 0
                else:
                    placement_index == 0

                # save the current position as a path waypoint to follow in the future
                procedure_waypoints.insert(placement_index, controller.cartesian_position_history[0][1])

                # set the next state of the procedure
                previous_procedure_state = procedure_state
                procedure_state = SET_GOAL
                

        if procedure_state == SET_GOAL:

            # if the current objective of the procedure is to find waypoints, 
            # set goal point as offset from current position

            if current_objective == WAYPOINT_FINDING:
                
                # set direction of offset based on current direction of motion
                x_offset = controller.standard_scan_step * current_direction

                # calculate new goal position
                controller.move_goal = controller.cartesian_position_history[0][1] + x_offset

            elif current_objective == SCANNING:

                # check to make sure there are more waypoints to travel to
                if len(procedure_waypoints) > 0:

                    # pop out the last waypoint as the goal position
                    controller.move_goal = procedure_waypoints.pop(-1)
                
                else:

                    # exit the procedure
                    current_objective = EXITING
                    controller.move_goal = procedure_origin
            else:
                # set the new goal position as the origin
                controller.move_goal = procedure_origin
            
            # set the next state of the procedure
            previous_procedure_state = procedure_state
            procedure_state = MOVE_TO_GOAL

        if procedure_state == MOVE_TO_GOAL:
            
            if sqrt(sum(controller.move_goal - controller.cartesian_position_history[0] ** 2)) > controller.acceptable_cartesian_error: 
            
                # create a message to send control velocities
                velocity = TwistStamped()

                # calculate positional control inputs
                positional_control_inputs = controller.cartesian_position_calculate_control_input()
                
                # calculate force control inputs = calculate_force_control_inputs
                force_control_inputs = controller.force_error_calculate_control_input()

                velocity.twist.linear.x = positional_control_inputs[0] + force_control_inputs[0]
                velocity.twist.linear.y = positional_control_inputs[1] + force_control_inputs[1]
                velocity.twist.linear.z = positional_control_inputs[2] + force_control_inputs[2]

            else:

                # create an empty message
                velocity = TwistStamped()

                # save the previous state
                previous_procedure_state = previous_procedure_state

                # if procedure is complete
                if current_objective == EXITING:
                    procedure_state = EXIT_PROCEDURE

                # if finding waypoints
                if current_objective == WAYPOINT_FINDING:
                    procedure_state = CHECK_FOR_THYROID

                # if scanning for images
                if current_objective == SCANNING:
                    procedure_state == SAVE_IMAGE
            
            # Publish the velocity message
            controller.velocity_publisher.publish(velocity)

        if procedure_state == CHECK_FOR_THYROID:

            # if the thyroid is in the image, center the image on the thyroid
            if controller.thyroid_in_image_status:

                previous_procedure_state = procedure_state
                procedure_state = CENTER_IMAGE

            else:
                
                # save the previous procedure state
                previous_procedure_state = procedure_state
                
                # if looking for waypoints and heading towards the torso,
                # reverse direction and head back to the origin
                if current_objective == WAYPOINT_FINDING and current_direction == DIRECTION_TORSO:
                    current_direction = DIRECTION_HEAD
                    procedure_state = MOVE_TO_GOAL

                    # set the next movement goal as the origin
                    controller.move_goal = procedure_origin + controller.standard_scan_step * current_direction

                # if looking for waypoints and heading towards the head,
                # switch to scanning at each discovered waypoint and reverse direction
                elif current_objective == WAYPOINT_FINDING and current_direction == DIRECTION_HEAD:
                    current_direction = DIRECTION_TORSO
                    current_objective = SCANNING
                    procedure_state = SET_GOAL

        if procedure_state == SAVE_IMAGE:
            
            # save the current image

            # save the current robot pose

            # save the image mask

            # save the approximate area of the thyroid in image

            previous_procedure_state = procedure_state
            procedure_state = SET_GOAL

        if procedure_state == EXIT_PROCEDURE:
            
            procedure_complete_flag = True

    velocity = TwistStamped()
    controller.velocity_publisher.publish(velocity)