#!/usr/bin/env python3

"""
File containing the RobotControlNode class.
"""

# Import standard ros packages
from rospy import Time
from geometry_msgs.msg import TwistStamped, WrenchStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import Float64, Bool, UInt8, Float64MultiArray, MultiArrayDimension

# Import standard packages
from numpy import zeros, array, median, append, arange, linspace
from copy import copy
from scipy.spatial.transform import Rotation

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import Float64MultiArrayStamped, Float64Stamped, RegisteredData

# Import custom python packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *
from thyroid_ultrasound_robot_control_support.RobotConstants import *
from thyroid_ultrasound_robot_control_support.ControllerConstants import *
from thyroid_ultrasound_robot_control_support.BasicController import BasicController
from thyroid_ultrasound_robot_control_support.SurfaceController import SurfaceController
from thyroid_ultrasound_robot_control_support.Surface import Surface


class RobotControlNode(BasicNode):

    def __init__(self) -> None:

        # Add call to super class init
        super().__init__()

        # Define the max speed allowed for any linear control input
        self.lin_max_speed = 0.2  # m/s
        self.ang_max_speed = 0.01  # rad/s

        # Define flag to know when the image is centered
        self.is_image_centered = False

        # Define controller objects for each dimension
        # TODO - High - THESE CONTROLLERS NEED TO BE TUNED
        self.linear_x_controller = SurfaceController(p_gain=0.300, error_tolerance=0.007,
                                                     d_gain=0.0000,
                                                     i_gain=0.0000)  # x linear, position-based, error = meters
        self.linear_y_controller = BasicController(p_gain=0.1, error_tolerance=0.0002,
                                                   d_gain=0.000, i_gain=.000,
                                                   set_point=0.)  # y linear, image-based, error = meters
        self.linear_z_controller = BasicController(p_gain=.01, error_tolerance=0.100,
                                                   d_gain=.000, i_gain=.000)  # z linear, force-based, error = Newtons
        self.angular_x_controller = BasicController(p_gain=0.02, error_tolerance=0.100,
                                                    d_gain=0.00, i_gain=0,
                                                    set_point=0.)  # x rotation, image-based, error = degrees
        self.angular_y_controller = BasicController(p_gain=0.00, error_tolerance=1.000,
                                                    d_gain=0.00, i_gain=0)  # y rotation, not measured, error = degrees
        self.angular_z_controller = BasicController(p_gain=0.00, error_tolerance=1.000,
                                                    d_gain=0.00, i_gain=0)  # z rotation, not measured, error = degrees

        # Define arrays to store gain constants
        self.position_based_gains = array([.01, .01, .01, .01, .01, .01])
        self.force_based_gains = array([.01, .01, .005, .01, .01, .01])

        # Define variables to store data for force history
        self.force_history_length = 15
        self.force_history = [array([]), array([]), array([]), array([]), array([]), array([])]

        # Define control inputs to be used
        self.image_based_control_input = zeros(6)
        self.force_based_control_input = zeros(6)
        self.position_based_control_input = zeros(6)

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

        # Define a dictionary to store each individual transformation
        self.individual_transformations = {LINK_1: array([]),
                                           LINK_2: array([]),
                                           LINK_3: array([]),
                                           LINK_4: array([]),
                                           LINK_5: array([]),
                                           LINK_6: array([]),
                                           LINK_7: array([]),
                                           LINK_8: array([]),
                                           LINK_EE: array([]),
                                           }

        # Define a variable to save the current robot pose
        self.o_t_ee = None

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
        init_node('RobotControlNode')

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
        self.position_error_publisher = Publisher(RC_POSITION_ERROR, Float64Stamped, queue_size=1)
        self.position_goal_surface_publisher = Publisher(RC_POSITION_GOAL_SURFACE, Float64MultiArray, queue_size=1)
        self.position_goal_transform_publisher = StaticTransformBroadcaster()
        self.position_based_controller_use_publisher = Publisher(RC_POSITION_IN_USE, Bool, queue_size=1)
        self.position_based_control_input_publisher = Publisher(RC_POSITION_CONTROL_INPUT_EE, TwistStamped,
                                                                queue_size=1)

        # Define publishers for displaying information about the image control
        self.image_based_controller_use_publisher = Publisher(RC_IMAGE_IN_USE, Bool, queue_size=1)
        self.image_based_control_input_publisher = Publisher(RC_IMAGE_CONTROL_INPUT_EE, TwistStamped, queue_size=1)

        # Create control input subscribers
        Subscriber(RC_IMAGE_ERROR, TwistStamped, self.image_based_control_input_calculation)
        Subscriber(ROBOT_FORCE, WrenchStamped, self.robot_force_control_input_calculation)

        # Create goal state subscribers
        Subscriber(RC_FORCE_SET_POINT, Float64, self.force_set_point_callback)

        # Create command subscribers
        Subscriber(USE_IMAGE_FEEDBACK, Bool, self.use_image_feedback_command_callback)
        Subscriber(USE_POSE_FEEDBACK, Bool, self.use_pose_feedback_command_callback)
        Subscriber(USE_FORCE_FEEDBACK, Bool, self.use_force_feedback_command_callback)
        Subscriber(CREATE_TRAJECTORY, Float64, self.create_trajectory_command_callback)
        Subscriber(CLEAR_TRAJECTORY, Bool, self.clear_trajectory_command_callback)

        Subscriber(REGISTERED_DATA, RegisteredData, self.registered_data_received_callback)

        Subscriber(ROBOT_DERIVED_POSE, Float64MultiArrayStamped, self.robot_pose_callback)

        Subscriber(IS_IMAGE_EMPTY, Bool, self.image_based_patient_contact_callback)

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
        if self.o_t_ee is not None:
            # Calculate the control inputs relative to the base frame
            y_lin_output_in_o_frame = self.get_rotation_matrix_of_pose() @ array([[0], [y_lin_output], [0]])

            # Save the control inputs
            self.image_based_control_input = [
                y_lin_output_in_o_frame[0][0],  # x linear in end effector frame
                y_lin_output_in_o_frame[1][0],  # y linear in end effector frame
                y_lin_output_in_o_frame[2][0],  # z linear in end effector frame
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
        self.force_history[3] = append(self.force_history[3], data.wrench.torque.x)

        # Average the force history to find a more consistent force value
        self.robot_sensed_force[2] = round(float(median(self.force_history[2])), 2)
        self.robot_sensed_force[3] = round(float(median(self.force_history[3])), 2)

        # Create the message used to send the cleaned force data
        cleaned_force_message = WrenchStamped()

        # Add the time stamp to the cleaned force data
        cleaned_force_message.header.stamp = Time.now()

        # Assign the necessary fields in the message
        cleaned_force_message.wrench.force.z = self.robot_sensed_force[2]
        cleaned_force_message.wrench.torque.x = self.robot_sensed_force[3]

        # Publish the cleaned force message
        self.cleaned_force_publisher.publish(cleaned_force_message)

        # If the robot is currently contacting the patient,
        if self.in_contact_with_patient:

            # Calculate the control output relative to the end effector frame
            z_lin_input, z_lin_set_point_reached, z_lin_current_error = self.linear_z_controller.calculate_output(
                self.robot_sensed_force[2]
            )
            x_ang_input, x_ang_set_point_reached, x_ang_current_error = self.angular_x_controller.calculate_output(
                self.robot_sensed_force[3]
            )

            # Publish the control output relative to the end effector frame
            self.force_based_control_input_publisher.publish(self.create_twist_stamped_from_list([0, 0, z_lin_input,
                                                                                                  x_ang_input, 0, 0]))

            # Ensure that the transformation matrix to the end effector exists before using it
            if self.o_t_ee is not None:
                # Transform the output from the end effector frame to the origin frame
                lin_input_in_o_frame = self.get_rotation_matrix_of_pose() @ array([[0], [0], [z_lin_input]])
                ang_input_in_o_frame = self.get_rotation_matrix_of_pose() @ array([[x_ang_input], [0], [0]])

                # Save the control inputs for use in the main loop
                self.force_based_control_input = [
                    lin_input_in_o_frame[0][0],  # x linear in end effector frame
                    lin_input_in_o_frame[1][0],  # y linear in end effector frame
                    lin_input_in_o_frame[2][0],  # z linear in end effector frame
                    ang_input_in_o_frame[0][0],  # x angular in end effector frame
                    ang_input_in_o_frame[1][0],  # y angular in end effector frame
                    ang_input_in_o_frame[2][0],  # z angular in end effector frame
                ]

            # Publish the error of the force system
            self.force_based_error_publisher.publish(Float64([0, 0, z_lin_current_error, x_ang_current_error, 0, 0]))

        # Otherwise send no force control signal
        else:
            self.force_based_control_input = [0, 0, 0, 0, 0, 0]

    def robot_pose_control_input_calculation(self) -> None:
        """
        Calculate the robot pose based control input using the current pose and the pose goal
        """
        # Only compute if the pose goal exists
        if self.o_t_ee is not None:

            # Calculate the output based on the current distance to the surface
            x_lin_output, self.current_trajectory_set_point_reached, x_lin_current_error = \
                self.linear_x_controller.calculate_output(
                    array([self.o_t_ee[0][3], self.o_t_ee[1][3], self.o_t_ee[2][3]])
                )

            # Publish the control input relative to the end effector
            self.position_based_control_input_publisher.publish(self.create_twist_stamped_from_list([x_lin_output, 0, 0,
                                                                                                     0, 0, 0]))

            # Transform the output from the end effector frame to the origin frame
            x_lin_output_in_o_frame = self.get_rotation_matrix_of_pose() @ array([[x_lin_output], [0], [0]])

            # Update the position based control input
            self.position_based_control_input = [
                x_lin_output_in_o_frame[0][0],  # x linear
                x_lin_output_in_o_frame[1][0],  # y linear
                x_lin_output_in_o_frame[2][0],  # z linear
                0,  # x angular
                0,  # y angular
                0,  # z angular
            ]

            # Publish if the goal position was reached
            self.position_goal_reached_publisher.publish(Bool(self.current_trajectory_set_point_reached))
            position_error_msg = Float64Stamped()
            position_error_msg.data.data = x_lin_current_error
            position_error_msg.header.stamp = Time.now()
            self.position_error_publisher.publish(position_error_msg)

            # If the set point has been reached,
            if self.current_trajectory_set_point_reached and self.has_data_been_registered:
                # Note that the current set point has no longer been reached
                self.current_trajectory_set_point_reached = False

                # Delete the first coordinate in the trajectory
                self.current_trajectory = array([self.current_trajectory[0][1:],
                                                 self.current_trajectory[1][1:],
                                                 self.current_trajectory[2][1:],
                                                 ])

                # Update the current trajectory set point as necessary
                self.update_current_trajectory_set_point()

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
            self.current_trajectory = None
            self.current_trajectory_set_point = None
            self.linear_x_controller.update_set_point(None)

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
            num_points = round(travel_distance/min_distance_between_registered_scans)

            # Save a copy of the robot pose transformation to use to ensure data is not overwritten in the process
            local_pose_transformation = copy(self.o_t_ee)

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
            a = len(self.current_trajectory)
            pass

    # endregion
    ###########################

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
    def robot_pose_callback(self, msg: Float64MultiArrayStamped):
        """
        Pull the transformation matrix out of the message then calculate the control input based on the robot pose.
        """
        # Save the transformation matrix
        self.o_t_ee = array(msg.data.data).reshape((4, 4))

        # Calculate the corresponding control input
        self.robot_pose_control_input_calculation()

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
        transform_msg.header.frame_id = 'panda_link0'
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

        if self.use_pose_feedback_flag:
            control_input_array = control_input_array + self.position_based_control_input

        if self.use_image_feedback_flag:
            control_input_array = control_input_array + self.image_based_control_input

        if self.use_force_feedback_flag:
            control_input_array = control_input_array + self.force_based_control_input

        # Publish the in_use flags
        self.position_based_controller_use_publisher.publish(Bool(self.use_pose_feedback_flag))
        self.force_based_controller_use_publisher.publish(Bool(self.use_force_feedback_flag))
        self.image_based_controller_use_publisher.publish(Bool(self.use_image_feedback_flag))

        # Limit the control inputs allowed to be supplied to the robot
        for ii in range(3):
            if control_input_array[ii] > self.lin_max_speed:
                control_input_array[ii] = self.lin_max_speed
            if control_input_array[ii+3] > self.ang_max_speed:
                control_input_array[ii+3] = self.ang_max_speed

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

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        # Run the main function of the node
        node.main()

        # Wait to run the node again
        publishing_rate.sleep()
