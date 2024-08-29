#!/usr/bin/env python3

"""
File containing the RobotControlNode class.
"""
# TODO - Dream - Add proper logging through the BasicNode Class
# TODO - Dream - Add proper exceptions for everything
# TODO - Dream - Add controls to test a range of forces to find the best force for the result

# Import standard ros packages
from geometry_msgs.msg import TwistStamped, WrenchStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import Float64MultiArray
from armer_msgs.msg import ManipulatorState

# Import standard packages
from numpy import zeros, array, median, append, arange
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import Float64Stamped, ControllerStatus

# Import custom python packages
from thyroid_ultrasound_support.BasicNode import *
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

        # Allow an argument to be passed to the code that determines which mode is being used
        parser = ArgumentParser()
        parser.add_argument("--simulate_robot", "--sr", dest="simulate_robot", action="store_true", default=False,
                            help="Simulates force readings from robot when robot is being simulated")
        parser.add_argument("__name", default="")
        parser.add_argument("__log", default="")

        # Parse the arguments passed to the code
        passed_arguments = parser.parse_args()

        # Define the max speed allowed for any linear control input
        self.lin_max_speed = 0.05  # m/s
        self.ang_max_speed = 0.25  # rad/s

        # Define an overall speed factor that can be used to control the overall speed of the robot
        self.overall_speed_factor = 1.0

        # Define controller objects for each dimension
        self.linear_x_controller = SurfaceController(p_gain=1.5, error_tolerance=0.0005,  # 0.3, 0.007, 0.000, 0.0000
                                                     d_gain=0.0000,
                                                     i_gain=0.0000)  # x linear, position-based, error = meters
        self.linear_y_controller = BasicController(p_gain=0.0000500, error_tolerance=15,  # 0.1, 0.0002, 0.000, 0.000
                                                   d_gain=0.000, i_gain=.000,
                                                   set_point=0.)  # y linear, image-based, error = pixels
        self.linear_z_controller = BasicController(p_gain=0.01, error_tolerance=0.05,  # 0.011, 0.1, 0.75, 0.000
                                                   d_gain=0.005, i_gain=0.000,
                                                   max_output=0.02)  # z linear, force-based, error = Newtons
        self.angular_x_controller = BasicController(p_gain=1.5, error_tolerance=0.005,  # 0.02, 0.1, 0.000, 0.000
                                                    d_gain=0.00, i_gain=0.000,
                                                    set_point=0.)  # x rotation, image-based, error = no units
        self.angular_y_controller = BasicController(p_gain=0.01, error_tolerance=0.050,
                                                    d_gain=0.00, i_gain=0)  # y rotation, position-based, error = deg
        self.angular_z_controller = BasicController(p_gain=0.01, error_tolerance=0.050,
                                                    d_gain=0.00, i_gain=0)  # z rotation, position-based, error = deg

        # Save the controllers in a dictionary for easy reference
        self.controllers = {X_LINEAR_CONTROLLER: self.linear_x_controller,
                            Y_LINEAR_CONTROLLER: self.linear_y_controller,
                            Z_LINEAR_CONTROLLER: self.linear_z_controller,
                            X_ANGULAR_CONTROLLER: self.angular_x_controller,
                            Y_ANGULAR_CONTROLLER: self.angular_y_controller,
                            Z_ANGULAR_CONTROLLER: self.angular_z_controller,
                            }

        # Define control inputs to be used
        self.experimental_noise_control_input = zeros(6)
        self.manual_control_input = zeros(6)

        # Define variables to store data about the force history
        self.force_history_length = 100
        self.force_history = [array([]), array([]), array([]), array([]), array([]), array([])]

        # Define variables to store the most recently received data
        self.newest_image_reading = 0.0
        self.newest_force_reading = zeros(6)
        self.newest_patient_contact_reading = 0.0

        # Define flag variables
        self.is_roi_in_image = False
        self.is_image_centered = False
        self.in_contact_with_patient = False
        self.image_is_frozen = False

        # Define override variables
        self.override_contact_with_patient = False
        self.simulate_robot_force_readings = passed_arguments.simulate_robot

        # Define variables to store commands sent by the controller
        self.use_image_feedback_flag = False
        self.use_pose_feedback_flag = False
        self.use_force_feedback_flag = False
        self.use_balancing_feedback_flag = False

        # Define a variable to save the current robot pose
        self.o_t_ee = None

        # Create a variable to store which PID controller should be published and modified
        self.selected_pid_controller = int(0)

        # Define a flag to note when the controller status should be published
        self.publish_controller_statuses = True

        # Initialize the node
        init_node(ROBOT_CONTROL)

        # Define publisher for cleaned force
        self.cleaned_force_publisher = Publisher(ROBOT_DERIVED_FORCE, WrenchStamped, queue_size=1)

        # Create robot cartesian velocity publisher
        self.cartesian_velocity_publisher = Publisher(ROBOT_CONTROL_INPUT, TwistStamped, queue_size=1)

        # Define publishers for displaying information about trajectory following
        self.position_goal_surface_publisher = Publisher(RC_POSITION_GOAL_SURFACE, Float64MultiArray, queue_size=1)
        self.position_goal_transform_publisher = StaticTransformBroadcaster()

        # Define publishers for publishing the statuses of the individual controllers
        self.linear_x_controller_status_publisher = Publisher(RC_LINEAR_X_CONTROLLER_STATUS,
                                                              ControllerStatus, queue_size=1)
        self.linear_y_controller_status_publisher = Publisher(RC_LINEAR_Y_CONTROLLER_STATUS,
                                                              ControllerStatus, queue_size=1)
        self.linear_z_controller_status_publisher = Publisher(RC_LINEAR_Z_CONTROLLER_STATUS,
                                                              ControllerStatus, queue_size=1)
        self.angular_x_controller_status_publisher = Publisher(RC_ANGULAR_X_CONTROLLER_STATUS,
                                                               ControllerStatus, queue_size=1)
        self.angular_y_controller_status_publisher = Publisher(RC_ANGULAR_Y_CONTROLLER_STATUS,
                                                               ControllerStatus, queue_size=1)
        self.angular_z_controller_status_publisher = Publisher(RC_ANGULAR_Z_CONTROLLER_STATUS,
                                                               ControllerStatus, queue_size=1)

        # Define publishers for when each controller has reached its set point
        self.position_lin_x_goal_reached_publisher = Publisher(RC_POSITION_GOAL_LIN_X_REACHED, Bool, queue_size=1)
        self.image_centering_goal_reached_publisher = Publisher(RC_IMAGE_CONTROL_GOAL_REACHED, Bool, queue_size=1)
        self.force_goal_reached_publisher = Publisher(RC_FORCE_CONTROL_GOAL_REACHED, Bool, queue_size=1)
        self.image_balancing_goal_reached_publisher = Publisher(RC_IMAGE_BALANCE_GOAL_REACHED, Bool, queue_size=1)
        self.position_ang_y_goal_reached_publisher = Publisher(RC_POSITION_GOAL_ANG_Y_REACHED, Bool, queue_size=1)
        self.position_ang_z_goal_reached_publisher = Publisher(RC_POSITION_GOAL_ANG_Z_REACHED, Bool, queue_size=1)

        # Define a publisher for publishing the position based error
        self.combined_position_error_publisher = Publisher(RC_POSITION_ERROR, TwistStamped, queue_size=1)

        # Create state subscribers
        Subscriber(ARMER_STATE, ManipulatorState, self.robot_state_callback)
        Subscriber(ROBOT_FORCE, WrenchStamped, self.robot_force_callback)
        Subscriber(RC_PATIENT_CONTACT_ERROR, Float64Stamped, self.patient_contact_error_callback)
        Subscriber(RC_IMAGE_ERROR, TwistStamped, self.image_error_callback)

        # Create control input subscribers
        Subscriber(RC_MANUAL_CONTROL_INPUT, TwistStamped, self.manual_control_input_callback)
        Subscriber(EXP_NOISE_VELOCITIES, TwistStamped, self.experimental_noise_calculation)

        # Create set point subscriber
        Subscriber(RC_FORCE_SET_POINT, Float64, self.force_set_point_callback)

        # Define subscribers for operational statuses
        Subscriber(IMAGE_PATIENT_CONTACT, Bool, self.image_based_patient_contact_callback)
        Subscriber(IMAGE_ROI_SHOWN, Bool, self.image_roi_shown_callback)
        Subscriber(IMAGE_FROZEN_STATUS, Bool, self.image_frozen_status_callback)

        # Define services for which control types to use
        Service(RC_USE_IMAGE_CENTERING, BoolRequest, self.use_image_feedback_command_handler)
        Service(RC_USE_POSE_CONTROL, BoolRequest, self.use_pose_feedback_command_handler)
        Service(RC_USE_FORCE_CONTROL, BoolRequest, self.use_force_feedback_command_handler)
        Service(RC_USE_IMAGE_BALANCING, BoolRequest, self.use_balancing_feedback_command_handler)

        # Define service for overriding patient contact signal
        Service(RC_OVERRIDE_PATIENT_CONTACT, BoolRequest, self.override_patient_contact_handler)

        # Define service for setting the overall speed of the robot
        Service(RC_OVERALL_ROBOT_SPEED, Float64Request, self.overall_speed_factor_handler)

        # Create services for trajectory management
        Service(RC_SET_TRAJECTORY_PITCH, Float64Request, self.set_trajectory_pitch_handler)
        Service(RC_SET_TRAJECTORY_YAW, Float64Request, self.set_trajectory_yaw_handler)
        Service(RC_SET_NEXT_WAYPOINT, Float64MultiArrayRequest, self.set_next_waypoint_handler)
        Service(RC_CLEAR_CURRENT_SET_POINTS, BoolRequest, self.clear_current_set_points_handler)
        Service(RC_SET_NEXT_FEATURE_WAYPOINT, TrajectoryWaypoint, self.set_next_feature_waypoint_handler())

        # Define service controlling the publishing of controller statuses
        Service(RC_PUBLISH_CONTROLLER_STATUSES, BoolRequest, self.publish_controller_statuses_handler)

        # Create services for setting the PID controllers
        Service(RC_VIEW_CONTROLLER_GAINS, ViewControllerGains, self.view_controller_gains_handler)
        Service(RC_SET_CONTROLLER_GAINS, SetControllerGains, self.set_controller_gains_handler)

    ############################
    # Individual state callbacks
    # region
    def image_error_callback(self, msg: TwistStamped):
        self.newest_image_reading = msg.twist.linear.x

    def robot_force_callback(self, msg: WrenchStamped):
        # Pop out an element from each history if too many have been stored
        for dimension, index in zip(self.force_history, arange(6)):
            if len(dimension) >= self.force_history_length:
                self.force_history[index] = dimension[1:]

        # Add in the new force value for each dimension
        if self.simulate_robot_force_readings:
            if self.linear_z_controller.set_point is not None:
                new_force_reading = self.linear_z_controller.set_point
            else:
                new_force_reading = 0
        else:
            new_force_reading = msg.wrench.force.z
        self.force_history[2] = append(self.force_history[2], new_force_reading)

        # Average the force history to find a more consistent force value
        self.newest_force_reading[2] = round(float(median(self.force_history[2])), 3)

    def patient_contact_error_callback(self, msg: Float64Stamped):
        self.newest_patient_contact_reading = msg.data.data

    def robot_state_callback(self, msg: ManipulatorState):
        """
        Pull the transformation matrix out of the message then calculate the control input based on the robot pose.
        """
        self.o_t_ee = convert_pose_to_transform_matrix(msg.ee_pose.pose)

        if self.simulate_robot_force_readings:
            self.robot_force_callback(msg.ee_wrench)

    # endregion
    ############################

    ##########################
    # Control input callbacks
    # region

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
        linear_movement_speed = 0.005
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

    # endregion
    ####################

    ###########################
    # Control scheme handlers
    # region
    def use_image_feedback_command_handler(self, req: BoolRequestRequest):
        """
        Update the use_image_feedback_flag based on the command value given.
        """
        self.use_image_feedback_flag = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def use_pose_feedback_command_handler(self, req: BoolRequestRequest):
        """
        Update the use_pose_feedback_flag based on the command value given.
        """
        self.use_pose_feedback_flag = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def use_force_feedback_command_handler(self, req: BoolRequestRequest):
        """
        Update the use_force_feedback_flag based on the command value given.
        """
        self.use_force_feedback_flag = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def use_balancing_feedback_command_handler(self, req: BoolRequestRequest):
        """
        Update the use_balancing_feedback_flag based on the command value given.
        """
        self.use_balancing_feedback_flag = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def override_patient_contact_handler(self, req: BoolRequestRequest):
        """
        Update the override_contact_with_patient based on the command value given.
        """
        self.override_contact_with_patient = req.value
        return BoolRequestResponse(True, NO_ERROR)

    # endregion
    ###########################

    ##################
    # Trajectory handlers
    # region
    def set_next_waypoint_handler(self, msg: Float64MultiArrayRequestRequest):

        # Publish the message
        self.position_goal_surface_publisher.publish(msg.next_waypoint)

        # Create a new surface based on the given vertices and set that surface as the set-point for the controller
        self.linear_x_controller.update_set_point(
            Surface(array(msg.next_waypoint.data).reshape((3, 3)))
        )

        # Publish the current set point as a transform to visualize it in RViz
        self.publish_goal_surface_as_transform()

        return Float64MultiArrayRequestResponse(True, NO_ERROR)

    def set_trajectory_pitch_handler(self, msg: Float64RequestRequest):
        self.angular_y_controller.update_set_point(msg.value)
        return Float64RequestResponse(True, NO_ERROR)

    def set_trajectory_yaw_handler(self, msg: Float64RequestRequest):
        self.angular_z_controller.update_set_point(msg.value)
        return Float64RequestResponse(True, NO_ERROR)

    def clear_current_set_points_handler(self, req: BoolRequestRequest):
        if req.value:
            self.linear_x_controller.update_set_point(None)
            self.angular_y_controller.update_set_point(None)
            self.angular_z_controller.update_set_point(None)

        return BoolRequestResponse(True, NO_ERROR)

    def set_next_feature_waypoint_handler(self, req: TrajectoryWaypointRequest):
        return TrajectoryWaypointResponse(True, NO_ERROR)

    def publish_controller_statuses_handler(self, req: BoolRequestRequest):
        self.publish_controller_statuses = req.value
        return BoolRequestResponse(True, NO_ERROR)

    # endregion
    ##################

    #########################
    # Robot Control Callbacks
    # region
    def overall_speed_factor_handler(self, req: Float64RequestRequest):
        """
        Set the overall speed factor as long as the values are within a safe range between 0 and 1.3 non-inclusive.
        """
        if 0 < req.value < 1.3:
            self.overall_speed_factor = req.value
            return Float64RequestResponse(True, NO_ERROR)

        return Float64RequestResponse(False, 'Value was out of range.')

    # endregion
    #########################

    ######################
    # PID Tuning Callbacks
    # region
    def view_controller_gains_handler(self, req: ViewControllerGainsRequest):
        return ViewControllerGainsResponse(self.controllers[req.controller_id].p_gain,
                                           self.controllers[req.controller_id].i_gain,
                                           self.controllers[req.controller_id].d_gain,
                                           True, NO_ERROR)

    def set_controller_gains_handler(self, req: SetControllerGainsRequest):
        for channel, value in zip([P_GAIN, I_GAIN, D_GAIN],
                                  [req.p_gain, req.i_gain, req.d_gain]):
            self.controllers[req.controller_id].set_gain(channel, value)
        return SetControllerGainsResponse(True, NO_ERROR)

    # endregion
    ######################

    def image_based_patient_contact_callback(self, data: Bool):
        self.in_contact_with_patient = data.data

    def image_roi_shown_callback(self, msg: Bool):
        self.is_roi_in_image = msg.data

    def image_frozen_status_callback(self, msg: Bool):
        self.image_is_frozen = msg.data

    def publish_goal_surface_as_transform(self):

        goal_vertex = self.linear_x_controller.set_point.point_on_plane
        goal_rotation_matrix = array([self.o_t_ee[0][:3],
                                      self.o_t_ee[1][:3],
                                      self.o_t_ee[2][:3],
                                      ])
        goal_rotation_quaternion = Rotation.from_matrix(goal_rotation_matrix).as_quat()

        transform_msg = TransformStamped()
        transform_msg.header.stamp = Time.now()
        transform_msg.header.frame_id = 'fr3_link0'  # panda_link0
        transform_msg.child_frame_id = 'goal_surface'
        transform_msg.transform.translation.x = goal_vertex[0]
        transform_msg.transform.translation.y = goal_vertex[1]
        transform_msg.transform.translation.z = goal_vertex[2]
        transform_msg.transform.rotation.x = goal_rotation_quaternion[0]
        transform_msg.transform.rotation.y = goal_rotation_quaternion[1]
        transform_msg.transform.rotation.z = goal_rotation_quaternion[2]
        transform_msg.transform.rotation.w = goal_rotation_quaternion[3]

        self.position_goal_transform_publisher.sendTransform(transform_msg)

    def publish_cleaned_force(self):
        # Create the message used to send the cleaned force data
        cleaned_force_message = WrenchStamped()

        # Add the time stamp to the cleaned force data
        cleaned_force_message.header.stamp = Time.now()

        # Assign the necessary fields in the message
        cleaned_force_message.wrench.force.z = self.newest_force_reading[2]

        # Publish the cleaned force message
        self.cleaned_force_publisher.publish(cleaned_force_message)

    def main(self) -> None:
        """
        Calculates the correct control input based on the current error in the system and
        which control modes are active.
        """

        # Calculate each pose-based control input
        if self.o_t_ee is not None:
            # Calculate the current roll, pitch, and yaw of the pose
            roll, pitch, yaw = calc_rpy(self.o_t_ee[0:3, 0:3])

            # Calculate the output based on the current distance to the surface
            x_lin_output, x_lin_set_point_reached, x_lin_current_error = \
                self.linear_x_controller.calculate_output(
                    array([self.o_t_ee[0][3], self.o_t_ee[1][3], self.o_t_ee[2][3]]))
            y_ang_output, y_ang_set_point_reached, y_ang_current_error = self.angular_y_controller.calculate_output(
                pitch)
            z_ang_output, z_ang_set_point_reached, z_ang_current_error = self.angular_z_controller.calculate_output(
                yaw)

        # Otherwise set all their values equal to zero
        else:
            x_lin_output = 0
            x_lin_current_error = 0
            x_lin_set_point_reached = False
            y_ang_output = 0
            y_ang_current_error = 0
            y_ang_set_point_reached = False
            z_ang_output = 0
            z_ang_current_error = 0
            z_ang_set_point_reached = False

        # Calculate each non-pose-based control input
        y_lin_output, y_lin_set_point_reached, y_lin_current_error = self.linear_y_controller.calculate_output(
            self.newest_image_reading)
        z_lin_output, z_lin_set_point_reached, z_lin_current_error = self.linear_z_controller.calculate_output(
            self.newest_force_reading[2])
        x_ang_output, x_ang_set_point_reached, x_ang_current_error = self.angular_x_controller.calculate_output(
            self.newest_patient_contact_reading)

        # Create blank array to store the final control input as an array
        control_input_array = zeros(6)

        # Add in the control inputs from each controller
        if self.o_t_ee is not None:
            if self.in_contact_with_patient or self.override_contact_with_patient:
                if self.use_pose_feedback_flag:
                    control_input_array = control_input_array + [x_lin_output, 0, 0, 0, -y_ang_output, -z_ang_output]
                if self.use_force_feedback_flag:
                    control_input_array[2] = control_input_array[2] + z_lin_output
            if self.in_contact_with_patient and not self.image_is_frozen:
                if self.use_balancing_feedback_flag:
                    control_input_array[3] = control_input_array[3] + x_ang_output
                if self.use_image_feedback_flag and self.is_roi_in_image:
                    control_input_array[1] = control_input_array[1] + y_lin_output

        # Add the control input from the noise-maker
        control_input_array = control_input_array + self.experimental_noise_control_input

        # Add the control input from the manual control
        control_input_array = control_input_array + self.manual_control_input

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
        # TODO - High - The line below this will need to be changed for the use with the real robot
        control_input_message.header.frame_id = 'fr3_raster'  # 'fr3_EE'
        control_input_message.twist.linear.x = control_input_array[0]
        control_input_message.twist.linear.y = control_input_array[1]
        control_input_message.twist.linear.z = control_input_array[2]
        control_input_message.twist.angular.x = control_input_array[3]
        control_input_message.twist.angular.y = control_input_array[4]
        control_input_message.twist.angular.z = control_input_array[5]

        self.cartesian_velocity_publisher.publish(control_input_message)

        # Publish the status of each controller
        if self.publish_controller_statuses:
            self.linear_x_controller_status_publisher.publish(
                ControllerStatus(controller_name='Trajectory Controller', in_use=self.use_pose_feedback_flag,
                                 current_error=x_lin_current_error, output=x_lin_output,
                                 set_point_reached=x_lin_set_point_reached))
            self.linear_y_controller_status_publisher.publish(
                ControllerStatus(controller_name='Image Controller', in_use=self.use_image_feedback_flag,
                                 current_error=y_lin_current_error, output=y_lin_output,
                                 set_point_reached=y_lin_set_point_reached))
            self.linear_z_controller_status_publisher.publish(
                ControllerStatus(controller_name='Force Controller', in_use=self.use_force_feedback_flag,
                                 current_error=z_lin_current_error, output=z_lin_output,
                                 set_point_reached=z_lin_set_point_reached))
            self.angular_x_controller_status_publisher.publish(
                ControllerStatus(controller_name='Patient Contact Controller', in_use=self.use_balancing_feedback_flag,
                                 current_error=x_ang_current_error, output=x_ang_output,
                                 set_point_reached=x_ang_set_point_reached))
            self.angular_y_controller_status_publisher.publish(
                ControllerStatus(controller_name='Trajectory Controller', in_use=self.use_pose_feedback_flag,
                                 current_error=y_ang_current_error, output=y_ang_output,
                                 set_point_reached=y_ang_set_point_reached))
            self.angular_z_controller_status_publisher.publish(
                ControllerStatus(controller_name='Trajectory Controller', in_use=self.use_pose_feedback_flag,
                                 current_error=z_ang_current_error, output=z_ang_output,
                                 set_point_reached=z_ang_set_point_reached))

        # Publish the cleaned force
        self.publish_cleaned_force()

        # Publish the combined position error
        new_msg = TwistStamped()
        new_msg.header.stamp = Time.now()
        new_msg.twist.linear.x = x_lin_current_error
        new_msg.twist.angular.y = y_ang_current_error
        new_msg.twist.angular.z = z_ang_current_error
        self.combined_position_error_publisher.publish(new_msg)

        # Publish the goal reached status for each controller
        self.position_lin_x_goal_reached_publisher.publish(x_lin_set_point_reached)
        self.image_centering_goal_reached_publisher.publish(y_lin_set_point_reached)
        self.force_goal_reached_publisher.publish(z_lin_set_point_reached)
        self.image_balancing_goal_reached_publisher.publish(x_ang_set_point_reached)
        self.position_ang_y_goal_reached_publisher.publish(y_ang_set_point_reached)
        self.position_ang_z_goal_reached_publisher .publish(z_ang_set_point_reached)


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
