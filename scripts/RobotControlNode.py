#!/usr/bin/env python3

"""
File containing the RobotControlNode class.
"""
# TODO - Dream - Fin-tune each of the PID controllers
# TODO - Dream - Add proper logging through the BasicNode Class
# TODO - Dream - Add proper exceptions for everything
# TODO - Dream - Add controls to test a range of forces to find the best force for the result
# TODO - High - Test and tune controllers maintaining the starting pose of the probe around the Y and Z axes

# Import standard ros packages
from geometry_msgs.msg import TwistStamped, WrenchStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import UInt8, Float64MultiArray, MultiArrayDimension
from franka_msgs.msg import FrankaState
from armer_msgs.msg import ManipulatorState

# Import standard packages
from numpy import zeros, array, median, append, arange, linspace, identity, transpose
from copy import copy
from scipy.spatial.transform import Rotation

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import Float64Stamped, RegisteredDataMsg

# Import custom python packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *
from thyroid_ultrasound_robot_control_support.RobotConstants import *
from thyroid_ultrasound_robot_control_support.Controllers.ControllerConstants import *
from thyroid_ultrasound_robot_control_support.Controllers.BasicController import BasicController
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.SurfaceController import \
    SurfaceController
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.Surface import Surface
from thyroid_ultrasound_robot_control_support.Helpers.calc_rpy import calc_rpy
from thyroid_ultrasound_robot_control_support.Helpers.convert_pose_to_transform_matrix import \
    convert_pose_to_transform_matrix


class RobotControlNode(BasicNode):

    def __init__(self) -> None:

        # Add call to super class init
        super().__init__()

        # Define the max speed allowed for any linear control input
        self.lin_max_speed = 0.05  # m/s
        self.ang_max_speed = 0.25  # rad/s

        # Define an overall speed factor that can be used to control the overall speed of the robot
        self.overall_speed_factor = 1.0

        # Define flag to know when the image is centered
        self.is_image_centered = False

        # Define controller objects for each dimension
        self.linear_x_controller = SurfaceController(p_gain=0.2, error_tolerance=0.007,  # 0.3, 0.007, 0.000, 0.0000
                                                     d_gain=0.0000,
                                                     i_gain=0.0000)  # x linear, position-based, error = meters
        self.linear_y_controller = BasicController(p_gain=0.100, error_tolerance=0.0002,  # 0.1, 0.0002, 0.000, 0.000
                                                   d_gain=0.000, i_gain=.000,
                                                   set_point=0.)  # y linear, image-based, error = meters
        self.linear_z_controller = BasicController(p_gain=0.011, error_tolerance=0.035,  # 0.011, 0.1, 0.75, 0.000
                                                   d_gain=0.050, i_gain=0.000,
                                                   max_output=0.02)  # z linear, force-based, error = Newtons
        self.angular_x_controller = BasicController(p_gain=0.25, error_tolerance=0.005,  # 0.02, 0.1, 0.000, 0.000
                                                    d_gain=0.00, i_gain=0.000,
                                                    set_point=0.)  # x rotation, image-based, error = no units
        self.angular_y_controller = BasicController(p_gain=0.01, error_tolerance=0.050,
                                                    d_gain=0.00, i_gain=0)  # y rotation, position-based, error = deg
        self.angular_z_controller = BasicController(p_gain=0.01, error_tolerance=0.050,
                                                    d_gain=0.00, i_gain=0)  # z rotation, position-based, error = deg

        # Define the physical features of the end effector
        self.end_effector_mass = 0.694  # kg
        self.end_effector_center_of_mass = array([[0], [0], [0.095]])  # meters

        # Define arrays to store gain constants
        self.position_based_gains = array([.01, .01, .01, .01, .01, .01])
        self.force_based_gains = array([.01, .01, .005, .01, .01, .01])

        # Define variables to store data for force history
        self.force_history_length = 100
        self.force_history = [array([]), array([]), array([]), array([]), array([]), array([])]

        # Define control inputs to be used
        self.image_based_control_input = zeros(6)
        self.force_based_control_input = zeros(6)
        self.position_based_control_input = zeros(6)
        self.patient_contact_based_control_input = zeros(6)
        self.experimental_noise_control_input = zeros(6)
        self.manual_control_input = zeros(6)

        # Define variables to store relevant robot information
        self.robot_sensed_force = zeros(6)
        self.current_pose = zeros(6)

        # Define variables to store desired motion
        self.goal_force = zeros(6)

        # Define variables to store commands sent by the controller
        self.stop_motion_command = False
        self.use_image_feedback_flag = False
        self.use_pose_feedback_flag = False
        self.use_force_feedback_flag = False
        self.use_balancing_feedback_flag = False

        # Define a dictionary to store each individual transformation
        # self.individual_transformations = {LINK_1: array([]),
        #                                    LINK_2: array([]),
        #                                    LINK_3: array([]),
        #                                    LINK_4: array([]),
        #                                    LINK_5: array([]),
        #                                    LINK_6: array([]),
        #                                    LINK_7: array([]),
        #                                    LINK_8: array([]),
        #                                    LINK_EE: array([]),
        #                                    }

        # Define a variable to save the current robot pose
        self.o_t_ee = None

        # Define a variable to store the robot rotation matrix
        # self.robot_rotation_matrix = identity(3)

        # Create a variable to store which PID controller should be published and modified
        self.selected_pid_controller = int(0)

        # Define a variable to save the currently calculated trajectory
        self.current_trajectory = None

        # Define a variable to store the current trajectory set point
        self.current_trajectory_set_point = None

        # Define a variable to store if the current set-point has been reached
        self.current_trajectory_set_point_reached = False

        # Define a variable to store if a registered data set has been produced
        self.has_data_been_registered = False

        # Define a variable to store if the robot is currently in contact with the patient
        self.in_contact_with_patient = False

        # Initialize the node
        init_node(ROBOT_CONTROL)

        # Define publisher for cleaned force
        self.cleaned_force_publisher = Publisher(ROBOT_DERIVED_FORCE, WrenchStamped, queue_size=1)

        # Create robot cartesian velocity publisher
        self.cartesian_velocity_publisher = Publisher(ROBOT_CONTROL_INPUT, TwistStamped, queue_size=1)

        # Create a publisher for displaying the force error
        self.force_based_error_publisher = Publisher(RC_FORCE_ERROR, Float64, queue_size=1)
        self.force_based_controller_use_publisher = Publisher(RC_FORCE_IN_USE, Bool, queue_size=1)
        self.force_based_control_input_publisher = Publisher(RC_FORCE_CONTROL_INPUT_EE, TwistStamped, queue_size=1)

        # Define publishers for displaying information about trajectory following
        self.position_goal_reached_publisher = Publisher(RC_POSITION_GOAL_REACHED, Bool, queue_size=1)
        self.position_error_publisher = Publisher(RC_POSITION_ERROR, TwistStamped, queue_size=1)
        self.position_goal_surface_publisher = Publisher(RC_POSITION_GOAL_SURFACE, Float64MultiArray, queue_size=1)
        self.position_goal_transform_publisher = StaticTransformBroadcaster()
        self.position_based_controller_use_publisher = Publisher(RC_POSITION_IN_USE, Bool, queue_size=1)
        self.position_based_control_input_publisher = Publisher(RC_POSITION_CONTROL_INPUT_EE, TwistStamped,
                                                                queue_size=1)
        self.trajectory_complete_publisher = Publisher(RC_TRAJECTORY_COMPLETE, Bool, queue_size=1)

        # Define publishers for displaying information about the image control
        self.image_based_controller_use_publisher = Publisher(RC_IMAGE_IN_USE, Bool, queue_size=1)
        self.image_based_control_input_publisher = Publisher(RC_IMAGE_CONTROL_INPUT_EE, TwistStamped, queue_size=1)

        # Define publishers for displaying information about tracking patient contact
        self.patient_contact_controller_use_publisher = Publisher(RC_PATIENT_CONTACT_IN_USE, Bool, queue_size=1)
        self.patient_contact_control_input_publisher = Publisher(RC_PATIENT_CONTACT_CONTROL_INPUT_EE, TwistStamped,
                                                                 queue_size=1)

        # Create control input subscribers
        Subscriber(RC_IMAGE_ERROR, TwistStamped, self.image_based_control_input_calculation)
        Subscriber(ROBOT_FORCE, WrenchStamped, self.robot_force_control_input_calculation)
        Subscriber(RC_PATIENT_CONTACT_ERROR, Float64Stamped, self.patient_contact_control_input_calculation)
        Subscriber(EXP_NOISE_VELOCITIES, TwistStamped, self.experimental_noise_calculation)

        # Create goal state subscribers
        Subscriber(RC_FORCE_SET_POINT, Float64, self.force_set_point_callback)

        # Create command subscribers
        Subscriber(USE_IMAGE_FEEDBACK, Bool, self.use_image_feedback_command_callback)
        Subscriber(USE_POSE_FEEDBACK, Bool, self.use_pose_feedback_command_callback)
        Subscriber(USE_FORCE_FEEDBACK, Bool, self.use_force_feedback_command_callback)
        Subscriber(USE_BALANCING_FEEDBACK, Bool, self.use_balancing_feedback_command_callback)

        Subscriber(CREATE_TRAJECTORY, Float64, self.create_trajectory_command_callback)
        Subscriber(CLEAR_TRAJECTORY, Bool, self.clear_trajectory_command_callback)

        Subscriber(REGISTERED_DATA_REAL_TIME, RegisteredDataMsg, self.registered_data_received_callback)

        Subscriber(IMAGE_PATIENT_CONTACT, Bool, self.image_based_patient_contact_callback)

        Subscriber(RC_OVERALL_ROBOT_SPEED, Float64, self.overall_speed_factor_callback)

        Subscriber(RC_MANUAL_CONTROL_INPUT, TwistStamped, self.manual_control_input_callback)

        # Create a subscriber to listen for the robot transformation
        Subscriber(ARMER_STATE, ManipulatorState, self.robot_state_callback)

        # Create publishers and subscribers to tune the PID controllers
        Subscriber(CONTROLLER_SELECTOR, UInt8, self.set_selected_controller_callback)
        Subscriber(P_GAIN_SETTING, Float64, self.set_p_gain_callback)
        Subscriber(I_GAIN_SETTING, Float64, self.set_i_gain_callback)
        Subscriber(D_GAIN_SETTING, Float64, self.set_d_gain_callback)
        self.p_gain_publisher = Publisher(P_GAIN_CURRENT, Float64, queue_size=1)
        self.i_gain_publisher = Publisher(I_GAIN_CURRENT, Float64, queue_size=1)
        self.d_gain_publisher = Publisher(D_GAIN_CURRENT, Float64, queue_size=1)

    ##########################
    # Calculate control inputs
    # region
    def image_based_control_input_calculation(self, data: TwistStamped) -> None:

        # Calculate the control input relative to the end effector using the image position error
        y_lin_output, y_lin_set_point_reached, y_lin_current_error = self.linear_y_controller.calculate_output(
            data.twist.linear.x
        )

        # Publish the control inputs relative to the end effector
        self.image_based_control_input_publisher.publish(self.create_twist_stamped_from_list([0, y_lin_output, 0,
                                                                                              0, 0, 0]))

        # Ensure that the transformation matrix to the end effector exists before using it
        if self.o_t_ee is not None and self.in_contact_with_patient:
            # Update the corresponding control input
            self.image_based_control_input = [
                0,  # x linear in end effector frame
                y_lin_output,  # y linear in end effector frame
                0,  # z linear in end effector frame
                0,  # x angular in end effector frame
                0,  # y angular in end effector frame
                0,  # z angular in end effector frame
            ]

    def robot_force_control_input_calculation(self, data: WrenchStamped) -> None:
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

        # Add in the new force value for each dimension
        self.force_history[2] = append(self.force_history[2], data.wrench.force.z)

        # Average the force history to find a more consistent force value
        self.robot_sensed_force[2] = round(float(median(self.force_history[2])), 3)

        # Create the message used to send the cleaned force data
        cleaned_force_message = WrenchStamped()

        # Add the time stamp to the cleaned force data
        cleaned_force_message.header.stamp = Time.now()

        # Assign the necessary fields in the message
        cleaned_force_message.wrench.force.z = self.robot_sensed_force[2]

        # Publish the cleaned force message
        self.cleaned_force_publisher.publish(cleaned_force_message)

        # If the robot is currently contacting the patient,
        if self.in_contact_with_patient:

            # Calculate the control output relative to the end effector frame
            z_lin_input, z_lin_set_point_reached, z_lin_current_error = self.linear_z_controller.calculate_output(
                self.robot_sensed_force[2]
            )

            # Publish the control output relative to the end effector frame
            self.force_based_control_input_publisher.publish(self.create_twist_stamped_from_list([0, 0, z_lin_input,
                                                                                                  0, 0, 0]))

            # Ensure that the transformation matrix to the end effector exists before using it
            if self.o_t_ee is not None:
                # Update the control inputs for use in the main loop
                self.force_based_control_input = [
                    0,  # x linear in end effector frame
                    0,  # y linear in end effector frame
                    z_lin_input,  # z linear in end effector frame
                    0,  # x angular in end effector frame
                    0,  # y angular in end effector frame
                    0,  # z angular in end effector frame
                ]

            # Publish the error of the force system
            self.force_based_error_publisher.publish(
                Float64(z_lin_current_error))  # Float64([0, 0, z_lin_current_error, 0, 0, 0]))

        # Otherwise send no force control signal
        else:
            self.force_based_control_input = [0, 0, 0, 0, 0, 0]

    def robot_pose_control_input_calculation(self) -> None:
        """
        Calculate the robot pose based control input using the current pose and the pose goal
        """
        # Only compute if the current pose exists
        if self.o_t_ee is not None:

            # Calculate the current roll, pitch, and yaw of the pose
            roll, pitch, yaw = calc_rpy(self.o_t_ee[0:3, 0:3])

            # Calculate the output based on the current distance to the surface
            x_lin_output, self.current_trajectory_set_point_reached, x_lin_current_error = \
                self.linear_x_controller.calculate_output(
                    array([self.o_t_ee[0][3], self.o_t_ee[1][3], self.o_t_ee[2][3]])
                )
            y_ang_output, y_ang_set_point_reached, y_ang_current_error = self.angular_y_controller.calculate_output(
                pitch
            )
            z_ang_output, z_ang_set_point_reached, z_ang_current_error = self.angular_z_controller.calculate_output(
                yaw
            )

            # Publish the control input relative to the end effector
            self.position_based_control_input_publisher.publish(self.create_twist_stamped_from_list([x_lin_output,
                                                                                                     0,
                                                                                                     0,
                                                                                                     0,
                                                                                                     y_ang_output,
                                                                                                     z_ang_output]))

            # If the robot is currently contacting the patient,
            if self.in_contact_with_patient:
                # Update the position based control input
                self.position_based_control_input = [
                    x_lin_output,  # x linear
                    0,  # y linear
                    0,  # z linear
                    0,  # x angular
                    y_ang_output,  # y angular
                    z_ang_output,  # z angular
                ]

            # Publish if the goal position was reached
            self.position_goal_reached_publisher.publish(Bool(self.current_trajectory_set_point_reached))
            position_error_msg = TwistStamped()
            position_error_msg.twist.linear.x = x_lin_current_error
            position_error_msg.twist.angular.y = y_ang_current_error
            position_error_msg.twist.angular.z = z_ang_current_error
            position_error_msg.header.stamp = Time.now()
            self.position_error_publisher.publish(position_error_msg)

            # If the set point has been reached,
            if self.current_trajectory_set_point_reached and self.has_data_been_registered:
                # Note that the current set point has no longer been reached
                self.current_trajectory_set_point_reached = False

                # Note that new data needs to be saved
                self.has_data_been_registered

                # Delete the first coordinate in the trajectory
                self.current_trajectory = array([self.current_trajectory[0][1:],
                                                 self.current_trajectory[1][1:],
                                                 self.current_trajectory[2][1:],
                                                 ])

                # Update the current trajectory set point as necessary
                self.update_current_trajectory_set_point()

    def patient_contact_control_input_calculation(self, msg: Float64Stamped) -> None:

        # Calculate the control input relative to the end effector using the image position error
        x_ang_output, x_ang_set_point_reached, x_ang_current_error = self.angular_x_controller.calculate_output(
            msg.data.data
        )

        # Publish the control inputs relative to the end effector
        self.patient_contact_control_input_publisher.publish(self.create_twist_stamped_from_list([0, 0, 0,
                                                                                                  x_ang_output, 0, 0]))

        # Ensure that the transformation matrix to the end effector exists before using it
        if self.o_t_ee is not None and self.in_contact_with_patient:
            # Update the appropriate control input
            self.patient_contact_based_control_input = [
                0,  # x linear in end effector frame
                0,  # y linear in end effector frame
                0,  # z linear in end effector frame
                x_ang_output,  # x angular in end effector frame
                0,  # y angular in end effector frame
                0,  # z angular in end effector frame
            ]

    def experimental_noise_calculation(self, msg: TwistStamped) -> None:

        # Only compute if the robot pose is known
        if self.o_t_ee is not None:
            # Update the proper control input
            self.experimental_noise_control_input = [
                0,  # x linear in end effector frame
                msg.twist.linear.y,  # y linear in end effector frame
                0,  # z linear in end effector frame
                0,  # x angular in end effector frame
                0,  # y angular in end effector frame
                0,  # z angular in end effector frame
            ]

    def manual_control_input_callback(self, msg: TwistStamped) -> None:

        # Get the velocity to give the message
        linear_movement_speed = 0.01
        pitch_angular_movement_speed = 0.05
        yaw_angular_movement_speed = 0.1

        # Only compute if the robot pose is known
        if self.o_t_ee is not None:

            # Calculate the control input for the pitch movement
            self.manual_control_input[4] = msg.twist.angular.y * pitch_angular_movement_speed

            # Calculate the control input for the yaw movement
            self.manual_control_input[5] = msg.twist.angular.z * yaw_angular_movement_speed

            # Only add the linear x input if pose feedback is not being used
            if not self.use_pose_feedback_flag:
                self.manual_control_input[0] = msg.twist.linear.x * linear_movement_speed

            # Only add the linear y input if image feedback is not being used
            if not self.use_image_feedback_flag:
                self.manual_control_input[1] = msg.twist.linear.y * linear_movement_speed

    # endregion
    ##########################

    ####################
    # Update goal values
    # region
    def force_set_point_callback(self, data: Float64) -> None:
        """
        Saves the force set-point to a local variable.

        Parameters
        ----------
        data
            The message containing the force set-point to save.
        """
        self.linear_z_controller.update_set_point(data.data)

    def update_current_trajectory_set_point(self):
        """
        Updates the current trajectory set point variable and the current trajectory variable depending on the
        level of completion of the trajectory.
        """

        # If the trajectory is not empty
        if self.current_trajectory.size > 0:

            # Save the next surface to travel to
            self.current_trajectory_set_point = array([self.current_trajectory[0][0],
                                                       self.current_trajectory[1][0],
                                                       self.current_trajectory[2][0],
                                                       ])

            # Create a new multi-dimension array message to transmit the goal surface information
            current_trajectory_set_point_message = Float64MultiArray()

            # Fill in the message with data from the current trajectory set point
            current_trajectory_set_point_message.layout.dim.append(MultiArrayDimension())
            current_trajectory_set_point_message.layout.dim[0].label = 'vectors'
            current_trajectory_set_point_message.layout.dim[0].size = self.current_trajectory_set_point.shape[0]
            current_trajectory_set_point_message.layout.dim[0].stride = self.current_trajectory_set_point.size
            current_trajectory_set_point_message.layout.dim.append(MultiArrayDimension())
            current_trajectory_set_point_message.layout.dim[1].label = 'dimensions'
            current_trajectory_set_point_message.layout.dim[1].size = self.current_trajectory_set_point.shape[1]
            current_trajectory_set_point_message.layout.dim[1].stride = self.current_trajectory_set_point.shape[1]
            current_trajectory_set_point_message.data = self.current_trajectory_set_point.reshape(
                [self.current_trajectory_set_point.size])

            # Publish the message
            self.position_goal_surface_publisher.publish(current_trajectory_set_point_message)

            # Create a new surface based on the given vertices and set that surface as the set-point for the controller
            self.linear_x_controller.update_set_point(
                Surface(self.current_trajectory_set_point)
            )

            # Publish the current set point as a transform to visualize it in RViz
            self.publish_goal_surface_as_transform()

        # Otherwise clear the trajectory and the current set point
        else:
            if self.current_trajectory is not None:
                self.trajectory_complete_publisher.publish(Bool(True))
            self.current_trajectory = None
            self.current_trajectory_set_point = None
            self.linear_x_controller.update_set_point(None)
            self.angular_y_controller.update_set_point(None)
            self.angular_z_controller.update_set_point(None)

    # endregion
    ####################

    ###########################
    # Command message callbacks
    # region
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

        print("Command sent: " + str(data.data))

    def use_balancing_feedback_command_callback(self, data: Bool) -> None:
        """
        Update the use_balancing_feedback_flag based on the command value given.
        """
        if not self.use_balancing_feedback_flag == data.data:
            if data.data:
                print("Balancing feedback turned on.")
            else:
                print("Balancing feedback turned off.")
        self.use_balancing_feedback_flag = data.data

        print("Command sent: " + str(data.data))

    def create_trajectory_command_callback(self, data: Float64):
        """
        Create a trajectory using the given offset in the y-axis of the end effector.
        """

        # Do not try to create a trajectory unless the robot pose transformation is known
        if self.o_t_ee is not None:

            # Define the distance to travel and the number of points to generate along the way
            travel_distance = data.data
            min_distance_between_registered_scans = 3  # millimeters
            min_distance_between_registered_scans = min_distance_between_registered_scans / 1000  # converted to meters
            num_points = abs(round(travel_distance / min_distance_between_registered_scans))

            # Save a copy of the robot pose transformation to use to ensure data is not overwritten in the process
            local_pose_transformation = copy(self.o_t_ee)

            # Calculate the RPY of the current pose to use for the trajectory maintenance
            roll, pitch, yaw = calc_rpy(self.o_t_ee[0:3, 0:3])

            # Set the current pitch and yaw as the set-points for the angular controllers
            self.angular_y_controller.update_set_point(pitch)
            self.angular_z_controller.update_set_point(yaw)

            # Create linear vectors on each plane between the current position and a distance in the x-axis containing
            # a given number of points
            trajectory = array([linspace(start=array([0, 0, 0]), stop=array([travel_distance, 0, 0]), num=num_points),
                                linspace(start=array([0, 1, 0]), stop=array([travel_distance, 1, 0]), num=num_points),
                                linspace(start=array([0, 0, 1]), stop=array([travel_distance, 0, 1]), num=num_points)])

            # Transform each point coordinate into the origin frame of the robot
            ii = 0
            for vector in trajectory:
                jj = 0
                for coordinate in vector:
                    # Add a zero to the end of the coordinate to allow multiplication by the transformation matrix
                    temp_coordinate = append(coordinate, 1)

                    # Reshape the coordinate into a column vector
                    temp_coordinate = temp_coordinate.reshape((4, 1))

                    # Transform the coordinate into the origin frame of the robot
                    temp_coordinate = local_pose_transformation @ temp_coordinate

                    # Save the transformed coordinate into the trajectory
                    trajectory[ii][jj] = temp_coordinate[0:3].reshape(3)

                    jj = jj + 1
                ii = ii + 1

            # Save the generated trajectory as the current trajectory
            self.current_trajectory = trajectory

            # Set the current set point for the trajectory
            self.update_current_trajectory_set_point()

    def clear_trajectory_command_callback(self, data: Bool):
        """
        Clear the current trajectory from the node. NOT IMPLEMENTED
        """
        if data.data:
            # Clear the trajectory
            self.current_trajectory = array([array([]), array([]), array([])])

            # Update the trajectory
            self.update_current_trajectory_set_point()

    # endregion
    ###########################

    #########################
    # Robot Control Callbacks
    # region
    def overall_speed_factor_callback(self, data: Float64):
        """
        Set the overall speed factor as long as the values are within a safe range between 0 and 1.3 non-inclusive.
        """
        if 0 < data.data < 1.3:
            self.overall_speed_factor = data.data

    # endregion
    #########################

    ######################
    # PID Tuning Callbacks
    # region
    def set_selected_controller_callback(self, data: UInt8) -> None:

        # Save the selected controller
        self.selected_pid_controller = data.data

        # Publish the values of the selected controller
        self.publish_selected_controller_gains()

    def set_p_gain_callback(self, data: Float64) -> None:
        self.set_controller_gain(P_GAIN, data.data)

        # Publish the values of the selected controller
        self.publish_selected_controller_gains()

    def set_i_gain_callback(self, data: Float64) -> None:
        self.set_controller_gain(I_GAIN, data.data)

        # Publish the values of the selected controller
        self.publish_selected_controller_gains()

    def set_d_gain_callback(self, data: Float64) -> None:
        self.set_controller_gain(D_GAIN, data.data)

        # Publish the values of the selected controller
        self.publish_selected_controller_gains()

    def set_controller_gain(self, channel_selector: int, new_gain_value: float) -> None:

        if self.selected_pid_controller == X_LINEAR_CONTROLLER:
            self.linear_x_controller.set_gain(channel_selector, new_gain_value)
        elif self.selected_pid_controller == Y_LINEAR_CONTROLLER:
            self.linear_y_controller.set_gain(channel_selector, new_gain_value)
        elif self.selected_pid_controller == Z_LINEAR_CONTROLLER:
            self.linear_z_controller.set_gain(channel_selector, new_gain_value)
        elif self.selected_pid_controller == X_ANGULAR_CONTROLLER:
            self.angular_x_controller.set_gain(channel_selector, new_gain_value)
        elif self.selected_pid_controller == Y_ANGULAR_CONTROLLER:
            self.angular_y_controller.set_gain(channel_selector, new_gain_value)
        elif self.selected_pid_controller == Z_ANGULAR_CONTROLLER:
            self.angular_z_controller.set_gain(channel_selector, new_gain_value)
        else:
            raise Exception("Incorrect controller selected")

    def publish_selected_controller_gains(self) -> None:

        # Equate the selection to the correct controller object
        if self.selected_pid_controller == X_LINEAR_CONTROLLER:
            selected_controller = self.linear_x_controller
        elif self.selected_pid_controller == Y_LINEAR_CONTROLLER:
            selected_controller = self.linear_y_controller
        elif self.selected_pid_controller == Z_LINEAR_CONTROLLER:
            selected_controller = self.linear_z_controller
        elif self.selected_pid_controller == X_ANGULAR_CONTROLLER:
            selected_controller = self.angular_x_controller
        elif self.selected_pid_controller == Y_ANGULAR_CONTROLLER:
            selected_controller = self.angular_y_controller
        elif self.selected_pid_controller == Z_ANGULAR_CONTROLLER:
            selected_controller = self.angular_z_controller
        else:
            raise Exception("Incorrect controller selected")

        # Publish the current gain values of the selected controller
        self.p_gain_publisher.publish(Float64(selected_controller.p_gain))
        self.i_gain_publisher.publish(Float64(selected_controller.i_gain))
        self.d_gain_publisher.publish(Float64(selected_controller.d_gain))

    # endregion
    ######################

    #####################
    # Robot Pose Callback
    # region

    def get_rotation_matrix_of_pose(self):
        return array([self.o_t_ee[0][:3],
                      self.o_t_ee[1][:3],
                      self.o_t_ee[2][:3],
                      ])

    def get_translation_matrix_of_pose(self):
        return array([[self.o_t_ee[0][3]],
                      [self.o_t_ee[0][3]],
                      [self.o_t_ee[0][3]],
                      ])

    def robot_state_callback(self, msg: ManipulatorState):
        """
        Pull the transformation matrix out of the message then calculate the control input based on the robot pose.
        """
        self.o_t_ee = convert_pose_to_transform_matrix(msg.ee_pose.pose)
        self.robot_pose_control_input_calculation()

    # endregion
    #####################

    def image_based_patient_contact_callback(self, data: Bool):
        self.in_contact_with_patient = data.data

    def registered_data_received_callback(self, _):
        """
        Once a set of registered data has been received, update the flag only if the robot has already reached
        current trajectory set point.
        """

        # If the current set point has been reached
        if self.current_trajectory_set_point_reached:
            # Note that a registered data point has been received
            self.has_data_been_registered = True

    def publish_goal_surface_as_transform(self):

        goal_vertex = self.current_trajectory_set_point[0]
        goal_rotation_matrix = array([self.o_t_ee[0][:3],
                                      self.o_t_ee[1][:3],
                                      self.o_t_ee[2][:3],
                                      ])
        goal_rotation_quaternion = Rotation.from_matrix(goal_rotation_matrix).as_quat()

        transform_msg = TransformStamped()
        transform_msg.header.stamp = Time.now()
        transform_msg.header.frame_id = 'fr3_link0' # panda_link0
        transform_msg.child_frame_id = 'goal_surface'
        transform_msg.transform.translation.x = goal_vertex[0]
        transform_msg.transform.translation.y = goal_vertex[1]
        transform_msg.transform.translation.z = goal_vertex[2]
        transform_msg.transform.rotation.x = goal_rotation_quaternion[0]
        transform_msg.transform.rotation.y = goal_rotation_quaternion[1]
        transform_msg.transform.rotation.z = goal_rotation_quaternion[2]
        transform_msg.transform.rotation.w = goal_rotation_quaternion[3]

        self.position_goal_transform_publisher.sendTransform(transform_msg)

    @staticmethod
    def create_twist_stamped_from_list(input_array: list):

        # Create the new message
        new_msg = TwistStamped()

        # Assign the fields appropriately
        new_msg.twist.linear.x = input_array[0]
        new_msg.twist.linear.y = input_array[1]
        new_msg.twist.linear.z = input_array[2]
        new_msg.twist.angular.x = input_array[3]
        new_msg.twist.angular.y = input_array[4]
        new_msg.twist.angular.z = input_array[5]

        return new_msg

    def main(self) -> None:
        """
        Calculates the correct control input based on the current error in the system and
        which control modes are active.
        """

        # Create blank array to store the final control input as an array
        control_input_array = zeros(6)

        # Add in the control inputs from each form of control
        if self.use_pose_feedback_flag:
            control_input_array = control_input_array + self.position_based_control_input

        if self.use_image_feedback_flag:
            control_input_array = control_input_array + self.image_based_control_input

        if self.use_force_feedback_flag:
            control_input_array = control_input_array + self.force_based_control_input

        if self.use_balancing_feedback_flag:
            control_input_array = control_input_array + self.patient_contact_based_control_input

        # Add the control input from the noise-maker
        control_input_array = control_input_array + self.experimental_noise_control_input

        # Add the control input from the manual control
        control_input_array = control_input_array + self.manual_control_input

        # Publish the in_use flags
        self.position_based_controller_use_publisher.publish(Bool(self.use_pose_feedback_flag))
        self.force_based_controller_use_publisher.publish(Bool(self.use_force_feedback_flag))
        self.image_based_controller_use_publisher.publish(Bool(self.use_image_feedback_flag))

        # Apply the overall speed factor to the final control input
        control_input_array = [j * self.overall_speed_factor for j in control_input_array]

        # Set the standard adjustment factor
        adjustment_factor = 1.0

        # If any linear control input is bigger than allowed,
        if max(control_input_array[0:3]) > self.lin_max_speed:

            # Define the adjustment factor accordingly
            adjustment_factor = self.lin_max_speed / max(control_input_array[0:3])

        # If any angular control input is bigger than allowed,
        if max(control_input_array[3:]) > self.ang_max_speed:

            # Adjust the adjustment factor accordingly
            adjustment_factor = adjustment_factor * self.ang_max_speed / max(control_input_array[3:])

        # Apply the adjustment to the control input
        control_input_array = [adjustment_factor * value for value in control_input_array]

        # Create a message and fill it with the desired control input
        control_input_message = TwistStamped()
        control_input_message.header.frame_id = 'fr3_EE'
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
    publishing_rate = Rate(300)  # hz

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        # Run the main function of the node
        node.main()

        # Wait to run the node again
        publishing_rate.sleep()
