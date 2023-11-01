#!/usr/bin/env python3

"""
File containing the RobotControlNode class.
"""

# Import standard ros packages
from rospy import is_shutdown, init_node, Rate, Publisher, Subscriber, Time
from geometry_msgs.msg import TwistStamped, WrenchStamped, TransformStamped, Transform
from tf import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64, Bool, UInt8, Float64MultiArray, MultiArrayDimension

# Import standard packages
from numpy import zeros, array, median, append, arange, delete, sum, dot, sqrt, cross, identity, linspace
from copy import copy
from scipy.spatial.transform import Rotation

# Define constants for each controller and controller channel
P_GAIN: int = int(0)
I_GAIN: int = int(1)
D_GAIN: int = int(2)

X_LINEAR_CONTROLLER: int = int(0)
Y_LINEAR_CONTROLLER: int = int(1)
Z_LINEAR_CONTROLLER: int = int(2)
X_ANGULAR_CONTROLLER: int = int(3)
Y_ANGULAR_CONTROLLER: int = int(4)
Z_ANGULAR_CONTROLLER: int = int(5)

# Define constants for the names of each link transformation
LINK_1: str = 'panda_link1'
LINK_2: str = 'panda_link2'
LINK_3: str = 'panda_link3'
LINK_4: str = 'panda_link4'
LINK_5: str = 'panda_link5'
LINK_6: str = 'panda_link6'
LINK_7: str = 'panda_link7'
LINK_8: str = 'panda_link8'
LINK_EE: str = 'panda_EE'

# TODO - High - Verify position error is being properly received


class RobotControlNode:

    def __init__(self) -> None:

        # Define flag to know when the image is centered
        self.is_image_centered = False

        # Define controller objects for each dimension
        # TODO - High - THESE CONTROLLERS NEED TO BE TUNED
        # TODO - Medium - Build some form of logging of values to aid in tuning
        self.linear_x_controller = SurfaceController(p_gain=0.300, error_tolerance=0.007,
                                                     d_gain=0.0000,
                                                     i_gain=0.0000)  # x linear, position-based, error = meters
        self.linear_y_controller = BasicController(p_gain=0.01, error_tolerance=0.003,
                                                   d_gain=0.000, i_gain=.000,
                                                   set_point=0.)  # y linear, image-based, error = meters
        self.linear_z_controller = BasicController(p_gain=.01, error_tolerance=0.100,
                                                   d_gain=.000, i_gain=.000)  # z linear, force-based, error = Newtons
        self.angular_x_controller = BasicController(p_gain=0.0001, error_tolerance=0.500,
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

        # Initialize the node
        init_node('RobotControlNode')

        # Create transform subscribers to calculate the current robot pose
        Subscriber('tf', TFMessage, self.create_transformation_callback)
        Subscriber('tf_static', TFMessage, self.read_static_transformation_callback)

        # Create control input subscribers
        Subscriber('/image_control/distance_to_centroid', TwistStamped, self.image_based_control_input_calculation)
        Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.robot_force_control_input_calculation)

        # Create goal state subscribers
        Subscriber('/force_control/set_point', Float64, self.force_set_point_callback)

        # Create command subscribers
        Subscriber('/command/use_image_feedback', Bool, self.use_image_feedback_command_callback)
        Subscriber('/command/use_pose_feedback', Bool, self.use_pose_feedback_command_callback)
        Subscriber('/command/use_force_feedback', Bool, self.use_force_feedback_command_callback)
        Subscriber('/command/create_trajectory', Float64, self.create_trajectory_command_callback)
        Subscriber('/command/clear_trajectory', Bool, self.clear_trajectory_command_callback)

        # Define publishers for robot information
        self.end_effector_transformation_publisher = Publisher('/O_T_EE', Float64MultiArray, queue_size=1)
        self.cleaned_force_publisher = Publisher('/force_control/sensed_force_cleaned', WrenchStamped, queue_size=1)

        # Create robot cartesian velocity publisher
        self.cartesian_velocity_publisher = Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

        # Create a publisher for displaying the force error
        self.force_based_error_publisher = Publisher('/force_control/error', Float64, queue_size=1)
        self.force_based_controller_use_publisher = Publisher('/force_control/in_use', Bool, queue_size=1)
        self.force_based_control_input_publisher = Publisher('/force_control/control_input_ee',
                                                             TwistStamped, queue_size=1)

        # Define publishers for displaying information about trajectory following
        self.position_goal_reached_publisher = Publisher('/position_control/goal_reached', Bool, queue_size=1)
        self.position_error_publisher = Publisher('/position_control/error', Float64, queue_size=1)
        self.position_goal_surface_publisher = Publisher('/position_control/goal_surface',
                                                         Float64MultiArray, queue_size=1)
        self.position_goal_transform_publisher = StaticTransformBroadcaster()
        self.position_based_controller_use_publisher = Publisher('/position_control/in_use', Bool, queue_size=1)
        self.position_based_control_input_publisher = Publisher('/position_control/control_input_ee',
                                                                TwistStamped, queue_size=1)

        # Define publishers for displaying information about the image control
        self.image_based_controller_use_publisher = Publisher('/image_control/in_use', Bool, queue_size=1)
        self.image_based_control_input_publisher = Publisher('/image_control/control_input_ee',
                                                             TwistStamped, queue_size=1)

        # Create publishers and subscribers to tune the PID controllers
        Subscriber('/tuning/controller', UInt8, self.set_selected_controller_callback)
        Subscriber('/tuning/setting/p_gain', Float64, self.set_p_gain_callback)
        Subscriber('/tuning/setting/i_gain', Float64, self.set_i_gain_callback)
        Subscriber('/tuning/setting/d_gain', Float64, self.set_d_gain_callback)
        self.p_gain_publisher = Publisher('/tuning/current/p_gain', Float64, queue_size=1)
        self.i_gain_publisher = Publisher('/tuning/current/i_gain', Float64, queue_size=1)
        self.d_gain_publisher = Publisher('/tuning/current/d_gain', Float64, queue_size=1)

    ##########################
    # Calculate control inputs
    # region
    def image_based_control_input_calculation(self, data: TwistStamped) -> None:

        # Calculate the control input relative to the end effector using the image position error
        y_lin_output, y_lin_set_point_reached, y_lin_current_error = self.linear_y_controller.calculate_output(
            data.twist.linear.x
        )
        x_ang_output, x_ang_set_point_reached, x_ang_current_error = self.angular_x_controller.calculate_output(
            data.twist.angular.y
        )

        # Publish the control inputs relative to the end effector
        self.image_based_control_input_publisher.publish(self.create_twist_stamped_from_list([0, y_lin_output, 0,
                                                                                              x_ang_output, 0, 0]))

        # Ensure that the transformation matrix to the end effector exists before using it
        if self.o_t_ee is not None:

            # Calculate the control inputs relative to the base frame
            y_lin_output_in_o_frame = self.get_rotation_matrix_of_pose()@array([[0], [y_lin_output], [0]])
            x_ang_output_in_o_frame = self.get_rotation_matrix_of_pose()@array([[x_ang_output], [0], [0]])

            # Save the control inputs
            self.image_based_control_input = [
                y_lin_output_in_o_frame[0][0],  # x linear
                y_lin_output_in_o_frame[1][0],  # y linear
                y_lin_output_in_o_frame[2][0],  # z linear
                x_ang_output_in_o_frame[0][0],  # x angular
                x_ang_output_in_o_frame[1][0],  # y angular
                x_ang_output_in_o_frame[2][0],  # z angular
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
        self.robot_sensed_force[2] = round(float(median(self.force_history[2])), 2)

        # Create the message used to send the cleaned force data
        cleaned_force_message = WrenchStamped()

        # Assign the necessary fields in the message
        cleaned_force_message.wrench.force.z = self.robot_sensed_force[2]

        # Publish the cleaned force message
        self.cleaned_force_publisher.publish(cleaned_force_message)

        # Calculate the control output relative to the end effector frame
        z_lin_output, z_lin_set_point_reached, z_lin_current_error = self.linear_z_controller.calculate_output(
            self.robot_sensed_force[2]
        )

        # Publish the control output relative to the end effector frame
        self.force_based_control_input_publisher.publish(self.create_twist_stamped_from_list([0, 0, z_lin_output,
                                                                                              0, 0, 0]))

        # Ensure that the transformation matrix to the end effector exists before using it
        if self.o_t_ee is not None:

            # Transform the output from the end effector frame to the origin frame
            output_in_o_frame = self.get_rotation_matrix_of_pose()@array([[0], [0], [z_lin_output]])

            self.force_based_control_input = [
                output_in_o_frame[0][0],  # x linear is not measured
                output_in_o_frame[1][0],  # y linear is not measured
                output_in_o_frame[2][0],  # z linear is measured
                0,  # x angular is not measured
                0,  # y angular is not measured
                0,  # z angular is not measured
            ]

        self.force_based_error_publisher.publish(Float64(z_lin_current_error))

    def robot_pose_control_input_calculation(self) -> None:
        """
        Calculate the robot pose based control input using the current pose and the pose goal
        """
        # Only compute if the pose goal exists
        if self.o_t_ee is not None:

            # Calculate the output based on the current distance to the surface
            x_lin_output, x_lin_set_point_reached, x_lin_current_error = self.linear_x_controller.calculate_output(
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
            self.position_goal_reached_publisher.publish(Bool(x_lin_set_point_reached))
            self.position_error_publisher.publish(Float64(x_lin_current_error))

            # If the set point has been reached,
            if x_lin_set_point_reached:
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
            current_trajectory_set_point_message.layout.dim.append(MultiArrayDimension)
            current_trajectory_set_point_message.layout.dim[0].label = 'vectors'
            current_trajectory_set_point_message.layout.dim[0].size = self.current_trajectory_set_point.shape[0]
            current_trajectory_set_point_message.layout.dim[0].stride = self.current_trajectory_set_point.size
            current_trajectory_set_point_message.layout.dim.append(MultiArrayDimension)
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
            num_points = 10

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

    #############################################################
    # Calculate the equivalent transformation to the end effector
    # region
    def read_static_transformation_callback(self, data: TFMessage):
        """
        Read the static transformation between of Link 7.
        """

        # Pull out single transform
        transform: TransformStamped = data.transforms[0]

        # Make sure the transformation is from Link 8
        if transform.child_frame_id == LINK_8:
            # Add the transformation to the dictionary
            self.create_transformation_from_transform(transform)

    def create_transformation_callback(self, data: TFMessage):
        """
        Calculate the transformation to the end effector based on the transforms given.
        """

        # For each transformation sent
        for transform in data.transforms:

            # Tell python what it is
            transform: TransformStamped

            # Pull out the name of the transformation
            transformation_name = transform.child_frame_id

            # If the name matches a key value
            if transformation_name in self.individual_transformations:
                # Add the transformation to the dictionary
                self.create_transformation_from_transform(transform)

        if len(data.transforms) == 7 and self.individual_transformations[LINK_EE].size > 0:
            # Define a variable to save the final transformation in
            o_t_ee = identity(4)

            # Iteratively calculate the final transformation through the transformation from each link
            for key in [LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, LINK_7, LINK_8, LINK_EE]:
                o_t_ee = o_t_ee @ self.individual_transformations[key]

            # Save the transformation for local use
            self.o_t_ee = o_t_ee

            # Define a new message object for publishing the transformation matrix
            o_t_ee_msg = Float64MultiArray()
            o_t_ee_msg.layout.dim.append(MultiArrayDimension)
            o_t_ee_msg.layout.dim[0].label = 'rows'
            o_t_ee_msg.layout.dim[0].size = 4
            o_t_ee_msg.layout.dim[0].stride = 16
            o_t_ee_msg.layout.dim.append(MultiArrayDimension)
            o_t_ee_msg.layout.dim[0].label = 'columns'
            o_t_ee_msg.layout.dim[0].size = 4
            o_t_ee_msg.layout.dim[0].stride = 4
            o_t_ee_msg.data = o_t_ee.reshape([16])

            self.end_effector_transformation_publisher.publish(o_t_ee_msg)

            # Calculate the control input based on the new robot pose
            self.robot_pose_control_input_calculation()

    def create_transformation_from_transform(self, transform: TransformStamped):
        """
        Add the given transform to dictionary of transformations after converting it to a homogeneous
        transformation matrix.
        """

        # Make sure the dictionary exists before trying to use it
        if self.individual_transformations is not None:
            # Pull out the translation vector and the rotation quaternion
            translation_vector = array([transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z])
            rotation_quaternion = array([transform.transform.rotation.x,
                                         transform.transform.rotation.y,
                                         transform.transform.rotation.z,
                                         transform.transform.rotation.w, ])

            # Calculate and store the homogeneous translation matrix
            self.individual_transformations[transform.child_frame_id] = self.create_homogeneous_transformation_matrix(
                translation_vector, rotation_quaternion
            )

    @staticmethod
    def create_homogeneous_transformation_matrix(v: array, q: array):
        """
        Create a homogeneous transformation matrix from a vertex and a quaternion.
        """

        # Calculate a rotation matrix from the quaternion
        r = Rotation.from_quat(q).as_matrix()

        # Assemble the homogeneous transformation matrix
        return array([
            [r[0][0], r[0][1], r[0][2], v[0]],
            [r[1][0], r[1][1], r[1][2], v[1]],
            [r[2][0], r[2][1], r[2][2], v[2]],
            [0, 0, 0, 1]
        ])

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
    #############################################################

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

        # Create a message and fill it with the desired control input
        control_input_message = TwistStamped()
        control_input_message.twist.linear.x = control_input_array[0]
        control_input_message.twist.linear.y = control_input_array[1]
        control_input_message.twist.linear.z = control_input_array[2]
        control_input_message.twist.angular.x = control_input_array[3]
        control_input_message.twist.angular.y = control_input_array[4]
        control_input_message.twist.angular.z = control_input_array[5]

        self.cartesian_velocity_publisher.publish(control_input_message)

    """def trajectory_goal_reached(self):
        # If the goal currently published goal has been reached and the current trajectory is not None
        if self.current_trajectory is not None:

            # Delete the first coordinate in the trajectory
            self.current_trajectory = array([self.current_trajectory[0][1:],
                                             self.current_trajectory[1][1:],
                                             self.current_trajectory[2][1:],
                                             ])

            # If the trajectory is not empty
            if self.current_trajectory.size > 0:

                # Generate the new trajectory message
                self.update_current_trajectory_set_point()

            # Else make both None
            else:
                self.current_trajectory = None
                self.current_trajectory_set_point = None"""


class BasicController:

    def __init__(self, p_gain: float, error_tolerance: float,
                 d_gain: float = None, i_gain: float = None,
                 set_point=None, min_output: float = None):

        self.error_history_length = 15
        self.error_history = array([])

        self.p_gain = p_gain
        self.d_gain = d_gain
        self.i_gain = i_gain

        self.set_point = set_point
        self.min_output = min_output

        self.error_tolerance = error_tolerance

    def update_set_point(self, new_set_point) -> None:
        self.set_point = new_set_point

    def set_gain(self, channel_selector: int, new_gain_value: float):

        if channel_selector == P_GAIN:
            self.p_gain = new_gain_value
        elif channel_selector == I_GAIN:
            self.i_gain = new_gain_value
        elif channel_selector == D_GAIN:
            self.d_gain = new_gain_value
        else:
            raise Exception("Incorrect channel selected.")

    def calculate_current_error(self, new_reading):
        """
        Calculates the current error based on the new reading.
        """
        return self.set_point - new_reading

    def calculate_output(self, new_reading):

        output = 0.

        if self.set_point is not None:

            # Calculate the current error
            current_error = self.calculate_current_error(new_reading)

            # Calculate the output based on the proportional gain
            output = output + (self.p_gain * current_error)

            if len(self.error_history) > 0 and self.p_gain is not None:
                # Calculate the output based on the derivative gain
                output = output + (self.d_gain * (current_error - self.error_history[-1]))

            if len(self.error_history) > 1 and self.i_gain is not None:
                # Calculate the output based on the integral gain
                output = output + (self.i_gain * sum(self.error_history))

            # Update the error history with the new error
            self.error_history = append(self.error_history, current_error)

            # Make sure that the error history is not too long
            if len(self.error_history) > self.error_history_length:
                self.error_history = delete(self.error_history, 0)

            if self.min_output is not None:
                if output < self.min_output:
                    output = 0

            return output, abs(current_error) <= self.error_tolerance, current_error

        return output, False, 0


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


class SurfaceController(BasicController):

    def __init__(self, p_gain: float, error_tolerance: float,
                 d_gain: float = None, i_gain: float = None,
                 set_point: Surface = None, min_output: float = None):
        super().__init__(p_gain, error_tolerance,
                         d_gain=d_gain, i_gain=i_gain,
                         set_point=set_point, min_output=min_output)

    def calculate_current_error(self, new_reading):
        if self.set_point is not None:
            return -self.set_point.distance_to_surface(new_reading)


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
