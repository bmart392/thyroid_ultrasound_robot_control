#!/usr/bin/env python3

"""
File containing the RobotControlNode class.
"""

# Import standard ros packages
from rospy import is_shutdown, init_node, Rate, Publisher, Subscriber
from geometry_msgs.msg import TwistStamped, WrenchStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from franka_msgs.msg import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, Bool, UInt8, Float64MultiArray, MultiArrayDimension

# Import standard packages
from numpy import zeros, array, median, append, arange, delete, sum, dot, sqrt, cross, identity
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
        self.linear_x_controller = BasicController(p_gain=0.10, error_tolerance=0.003,
                                                   d_gain=0.05, i_gain=1,
                                                   set_point=0.)  # x linear, image-based, error = meters
        self.linear_y_controller = SurfaceController(p_gain=0.150, error_tolerance=0.005,
                                                     d_gain=0.0005, i_gain=0.0005)  # y linear, position-based, error = meters
        self.linear_z_controller = BasicController(p_gain=0.10, error_tolerance=0.100,
                                                   d_gain=0.05, i_gain=1)  # z linear, force-based, error = Newtons
        self.angular_x_controller = BasicController(p_gain=0.00, error_tolerance=1.000,
                                                    d_gain=0.00, i_gain=0)  # x rotation, not measured, error = degrees
        self.angular_y_controller = BasicController(p_gain=0.10, error_tolerance=0.500,
                                                    d_gain=0.05, i_gain=1,
                                                    set_point=0.)  # y rotation, image-based, error = degrees
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
        self.O_T_EE = None

        # Initialize the node
        init_node('RobotControlNode')

        # Create control input subscribers
        Subscriber('/control_error/image_based', TwistStamped, self.image_based_control_error_callback)
        # Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.robot_sensed_force_callback)
        Subscriber('/franka_state_controller/F_ext', WrenchStamped, print("test_force"))
        Subscriber('/franka_state_controller/franka_states', FrankaState, self.robot_current_pose_callback)
        # Subscriber('tf', TFMessage, self.create_transformation_callback)
        Subscriber('tf', TFMessage, print("test"))
        # Subscriber('tf_static', TFMessage, self.read_static_transformation_callback)
        Subscriber('tf_static', TFMessage, print("test2"))

        # Create goal state subscribers
        # Subscriber('/position_control/goal_surface', Float64MultiArray, self.goal_surface_callback)
        Subscriber('/position_control/goal_surface', Float64MultiArray, print("hi"))
        Subscriber('/force_control/set_point', Float64, self.force_set_point_callback)

        # Create command subscribers
        Subscriber('/command/use_image_feedback', Bool, self.use_image_feedback_command_callback)
        Subscriber('/command/use_pose_feedback', Bool, self.use_pose_feedback_command_callback)
        Subscriber('/command/use_force_feedback', Bool, self.use_force_feedback_command_callback)

        # Create status publishers
        # self.was_last_goal_reached_status_publisher = Publisher('/status/goal_reached', Bool, queue_size=1)
        # self.current_pose_status_publisher = Publisher('/status/current_pose', PoseStamped, queue_size=1)

        self.end_effector_transformation_publisher = Publisher('/O_T_EE', Float64MultiArray, queue_size=1)

        # Create robot cartesian velocity publisher
        self.cartesian_velocity_publisher = Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

        # Define a publisher to publish if the image is centered
        # self.image_centered_publisher = Publisher('is_image_centered', Bool, queue_size=1)

        # Define a publisher to publish the cleaned reading of the sensed force
        self.cleaned_force_publisher = Publisher('/force_control/sensed_force_cleaned', WrenchStamped, queue_size=1)

        # Define a publisher to publish when the current goal has been reached
        self.position_goal_reached_publisher = Publisher('/position_control/goal_reached', Bool, queue_size=1)
        self.position_error_publisher = Publisher('/position_controller/error', Float64, queue_size=1)

        # Create publishers and subscribers to tune the PID controllers
        Subscriber('/tuning/controller', UInt8, self.set_selected_controller_callback)
        Subscriber('/tuning/setting/p_gain', Float64, self.set_p_gain_callback)
        Subscriber('/tuning/setting/i_gain', Float64, self.set_i_gain_callback)
        Subscriber('/tuning/setting/d_gain', Float64, self.set_d_gain_callback)
        self.p_gain_publisher = Publisher('/tuning/current/p_gain', Float64, queue_size=1)
        self.i_gain_publisher = Publisher('/tuning/current/i_gain', Float64, queue_size=1)
        self.d_gain_publisher = Publisher('/tuning/current/d_gain', Float64, queue_size=1)

        # Create a variable to store which PID controller should be published and modified
        self.selected_pid_controller = int(0)

    # Pull control inputs from message
    def image_based_control_error_callback(self, data: TwistStamped) -> None:

        x_lin_output, x_lin_set_point_reached = self.linear_x_controller.calculate_output(data.twist.linear.x)
        y_ang_output, y_ang_set_point_reached = self.angular_y_controller.calculate_output(data.twist.angular.y)

        self.image_based_control_input = [
            x_lin_output,  # x linear is measured
            0,  # y linear is not measured
            0,  # z linear is not measured
            0,  # x angular is not measured
            y_ang_output,  # y angular is measured
            0,  # z angular is not measured
        ]

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
        # self.robot_sensed_force[0] = 0  # mean(self.force_history[0])
        # self.robot_sensed_force[1] = 0  # mean(self.force_history[1])
        self.robot_sensed_force[2] = round(float(median(self.force_history[2])), 2)
        # self.robot_sensed_force[3] = 0  # mean(self.force_history[3])
        # self.robot_sensed_force[4] = 0  # mean(self.force_history[4])
        # self.robot_sensed_force[5] = 0  # mean(self.force_history[5])

        cleaned_force_message = WrenchStamped()
        # cleaned_force_message.wrench.force.x = self.robot_sensed_force[0]
        # cleaned_force_message.wrench.force.y = self.robot_sensed_force[1]
        cleaned_force_message.wrench.force.z = self.robot_sensed_force[2]
        # cleaned_force_message.wrench.torque.x = self.robot_sensed_force[3]
        # cleaned_force_message.wrench.torque.y = self.robot_sensed_force[4]
        # cleaned_force_message.wrench.torque.z = self.robot_sensed_force[5]

        self.cleaned_force_publisher.publish(cleaned_force_message)

        output, set_point_reached = self.linear_z_controller.calculate_output(self.robot_sensed_force[2])

        self.force_based_control_input = [
            0,  # x linear is not measured
            0,  # y linear is not measured
            output,  # z linear is measured
            0,  # x angular is not measured
            0,  # y angular is not measured
            0,  # z angular is not measured
        ]

    def robot_current_pose_callback(self, data: FrankaState) -> None:
        """
        Captures the current pose of the robot from messages sent by the robot controller.
        """
        if self.O_T_EE is not None:
            # Calculate the output based on the current distance to the surface
            y_lin_output, y_lin_set_point_reached = self.linear_y_controller.calculate_output(
                array([self.O_T_EE[0][3], self.O_T_EE[1][3], self.O_T_EE[2][3]])
            )

            # Update the position based control input
            self.position_based_control_input = [
                0,  # x linear is not measured
                y_lin_output,  # y linear is measured
                0,  # z linear is not measured
                0,  # x angular is not measured
                0,  # y angular is not measured
                0,  # z angular is not measured
            ]

            # Publish if the goal position was reached
            self.position_goal_reached_publisher.publish(Bool(y_lin_set_point_reached))
            self.position_error_publisher.publish(Float64(self.linear_y_controller.calculate_current_error(
                array([self.O_T_EE[0][3], self.O_T_EE[1][3], self.O_T_EE[2][3]])
            )))

            # If the set point has been reached, 
            if y_lin_set_point_reached:

                # Clear the set point from the controller to avoid race conditions
                self.linear_y_controller.update_set_point(None)

            # self.current_pose[0] = data.O_T_EE[3]
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

    def goal_surface_callback(self, data: Float64MultiArray):
        """
        Captures the goal surface sent to the node.
        """
        # Create a new surface based on the given vertices and set that surface as the set-point for the controller
        print("new set-point reached")
        self.linear_y_controller.update_set_point(
            Surface(array(data.data).reshape((data.layout.dim[0].size, data.layout.dim[1].size)))
        )

    def force_set_point_callback(self, data: Float64) -> None:
        """
        Saves the force set-point to a local variable.

        Parameters
        ----------
        data
            The message containing the force set-point to save.
        """
        self.linear_z_controller.update_set_point(data.data)

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

        # Create a message and fill it with the desired control input
        control_input_message = TwistStamped()
        control_input_message.twist.linear.x = control_input_array[0]
        control_input_message.twist.linear.y = control_input_array[1]
        control_input_message.twist.linear.z = control_input_array[2]
        control_input_message.twist.angular.x = control_input_array[3]
        control_input_message.twist.angular.y = control_input_array[4]
        control_input_message.twist.angular.z = control_input_array[5]

        self.cartesian_velocity_publisher.publish(control_input_message)

    def create_transformation_from_transform(self, transform: TransformStamped):

        # Make sure the dictionary exists before trying to use it
        if self.individual_transformations is not None:
                
            # Pull out the translation vector and the rotation quaternion
            translation_vector = array([transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z])
            rotation_quaternion = array([transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w,])
            
            # Calculate and store the homogeneous translation matrix
            self.individual_transformations[transform.child_frame_id] = self.create_homogeneous_transformation_matrix(
                translation_vector, rotation_quaternion
            )

    def read_static_transformation_callback(self, data: TFMessage):

        # Pull out single transform
        transform: TransformStamped = data.transforms[0]

        # Make sure the transformation is from Link 8
        if transform.child_frame_id == LINK_8:
            
            # Add the transformation to the dictionary
            self.create_transformation_from_transform(transform)

    def create_transformation_callback(self, data: TFMessage):

        # For each transformation sent
        for transform in data.transforms:

            # Tell python what it is
            transform: TransformStamped

            # Pull out the name of the transformation
            transformation_name = transform.child_frame_id

            # Define a variable to count the number of transformation sent

            # If the name matches a key value
            if transformation_name in self.individual_transformations:
                
                # Add the transformation to the dictionary
                self.create_transformation_from_transform(transform)
        
        if len(data.transforms) == 7 and self.individual_transformations[LINK_EE].size > 0:
            # Define a variable to save the final transformation in
            O_T_EE = identity(4) 

            # Iteratively calculate the final transformation through the transformation from each link
            for key in [LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, LINK_7, LINK_8, LINK_EE]:
                O_T_EE = O_T_EE@self.individual_transformations[key]

            # Save the transformation for local use
            self.O_T_EE = O_T_EE
            
            # Define a new message object for publishing the transformation matrix
            O_T_EE_msg = Float64MultiArray()
            O_T_EE_msg.layout.dim.append(MultiArrayDimension)
            O_T_EE_msg.layout.dim[0].label = 'rows'
            O_T_EE_msg.layout.dim[0].size = 4
            O_T_EE_msg.layout.dim[0].stride = 16
            O_T_EE_msg.layout.dim.append(MultiArrayDimension)
            O_T_EE_msg.layout.dim[0].label = 'columns'
            O_T_EE_msg.layout.dim[0].size = 4
            O_T_EE_msg.layout.dim[0].stride = 4
            O_T_EE_msg.data = O_T_EE.reshape([16])

            self.end_effector_transformation_publisher.publish(O_T_EE_msg)

    @staticmethod
    def create_homogeneous_transformation_matrix(v: array, q: array):

        r = Rotation.from_quat(q).as_matrix()

        return array([
                    [r[0][0], r[0][1], r[0][2], v[0]],
                    [r[1][0], r[1][1], r[1][2], v[1]],
                    [r[2][0], r[2][1], r[2][2], v[2]],
                    [0, 0, 0, 1]
        ])


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

            return output, abs(current_error) <= self.error_tolerance

        return output, False


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
            return self.set_point.distance_to_surface(new_reading)


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
