#!/usr/bin/env python3

"""
File containing the TrajectoryManagementNode class.
"""
# TODO - Dream - Add proper logging through the BasicNode Class
# TODO - Dream - Add proper exceptions for everything

# Import standard ROS packages
from armer_msgs.msg import ManipulatorState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# Import standard python packages
from numpy import copy, array, linspace, append
from rospy import sleep

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_services.srv import *

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Helpers.convert_pose_to_transform_matrix import \
    convert_pose_to_transform_matrix
from thyroid_ultrasound_robot_control_support.Helpers.calc_rpy import calc_rpy


class TrajectoryManagementNode(BasicNode):

    def __init__(self):

        super().__init__()

        # Define variable to store for the current pose of the robot
        self.current_pose = None

        # Define a variable to store the trajectory
        self.trajectory = None
        self.current_trajectory_set_point = None

        # Define the default spacing for trajectories
        self.min_distance_between_registered_scans = 1  # millimeters

        # Define flag variables
        self.is_patient_in_contact = False
        self.trajectory_waypoint_reached = False
        self.is_image_centered = False
        self.is_proper_force_applied = False
        self.is_image_balanced = False
        self.trajectory_pitch_goal_reached = False
        self.trajectory_yaw_goal_reached = False
        self.is_trajectory_paused = False
        self.data_registration_was_requested = False
        self.data_has_been_registered = False

        # Define override flag variables
        self.is_patient_in_contact_override = False
        self.is_proper_force_applied_override = False
        self.is_image_balanced_override = False
        self.is_image_centered_override = False
        self.registered_data_success_override = False

        # Initialize the ROS node
        init_node(TRAJECTORY_MANAGEMENT)

        # Define robot pose subscriber
        Subscriber(ARMER_STATE, ManipulatorState, self.current_pose_callback)

        # Define status subscribers
        Subscriber(IMAGE_PATIENT_CONTACT, Bool, self.is_patient_in_contact_callback)

        Subscriber(RC_POSITION_GOAL_LIN_X_REACHED, Bool, self.trajectory_waypoint_reached_callback)
        Subscriber(RC_IMAGE_CONTROL_GOAL_REACHED, Bool, self.is_image_centered_callback)
        Subscriber(RC_FORCE_CONTROL_GOAL_REACHED, Bool, self.is_proper_force_applied_callback)
        Subscriber(RC_IMAGE_BALANCE_GOAL_REACHED, Bool, self.is_image_balanced_callback)
        Subscriber(RC_POSITION_GOAL_ANG_Y_REACHED, Bool, self.trajectory_pitch_goal_reached_callback)
        Subscriber(RC_POSITION_GOAL_ANG_Z_REACHED, Bool, self.trajectory_yaw_goal_reached_callback)

        # Define override services
        Service(TM_OVERRIDE_PATIENT_CONTACT, BoolRequest, self.is_patient_in_contact_override_handler)
        Service(TM_OVERRIDE_FORCE_CONTROL, BoolRequest, self.is_proper_force_applied_override_handler)
        Service(TM_OVERRIDE_IMAGE_BALANCED, BoolRequest, self.is_image_balanced_override_handler)
        Service(TM_OVERRIDE_IMAGE_CENTERED, BoolRequest, self.is_image_centered_override_handler)
        Service(TM_OVERRIDE_DATA_REGISTERED, BoolRequest, self.registered_data_success_override_handler)

        # Define trajectory management services
        Service(TM_CREATE_TRAJECTORY, Float64Request, self.create_trajectory_handler)
        Service(TM_SET_TRAJECTORY_SPACING, Float64Request, self.set_trajectory_spacing_handler)
        Service(TM_CLEAR_TRAJECTORY, BoolRequest, self.clear_trajectory_handler)
        Service(TM_DATA_HAS_BEEN_REGISTERED, BoolRequest, self.data_has_been_registered_handler)

        # Define robot control service proxies
        self.set_trajectory_pitch_service = ServiceProxy(RC_SET_TRAJECTORY_PITCH, Float64Request)
        self.set_trajectory_yaw_service = ServiceProxy(RC_SET_TRAJECTORY_YAW, Float64Request)
        self.set_next_waypoint_service = ServiceProxy(RC_SET_NEXT_WAYPOINT, Float64MultiArrayRequest)
        self.clear_current_set_points_service = ServiceProxy(RC_CLEAR_CURRENT_SET_POINTS, BoolRequest)

        # Define image data registration service proxy
        self.register_new_data_service = ServiceProxy(IPR_REGISTER_NEW_DATA, BoolRequest)

        # Define the user interface proxies
        self.trajectory_complete_service = ServiceProxy(UI_TRAJECTORY_COMPLETE, BoolRequest)

    # Define status subscribers
    # region

    def current_pose_callback(self, msg: ManipulatorState):
        self.current_pose = convert_pose_to_transform_matrix(msg.ee_pose.pose)

    def is_patient_in_contact_callback(self, msg: Bool):
        self.is_patient_in_contact = msg.data

    def trajectory_waypoint_reached_callback(self, msg: Bool):
        self.trajectory_waypoint_reached = msg.data

    def is_image_centered_callback(self, msg: Bool):
        self.is_image_centered = msg.data

    def is_proper_force_applied_callback(self, msg: Bool):
        self.is_proper_force_applied = msg.data

    def is_image_balanced_callback(self, msg: Bool):
        self.is_image_balanced = msg.data

    def trajectory_pitch_goal_reached_callback(self, msg: Bool):
        self.trajectory_pitch_goal_reached = msg.data

    def trajectory_yaw_goal_reached_callback(self, msg: Bool):
        self.trajectory_yaw_goal_reached = msg.data

    # endregion

    # Define service handlers
    # region

    def is_patient_in_contact_override_handler(self, req: BoolRequestRequest):
        self.is_patient_in_contact_override = req.value
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    def is_proper_force_applied_override_handler(self, req: BoolRequestRequest):
        self.is_proper_force_applied_override = req.value
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    def is_image_balanced_override_handler(self, req: BoolRequestRequest):
        self.is_image_balanced_override = req.value
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    def is_image_centered_override_handler(self, req: BoolRequestRequest):
        self.is_image_centered_override = req.value
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    def registered_data_success_override_handler(self, req: BoolRequestRequest):
        self.registered_data_success_override = req.value
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    def data_has_been_registered_handler(self, req: BoolRequestRequest):
        self.data_has_been_registered = req.value
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    # endregion

    # Define service for creating trajectory
    def create_trajectory_handler(self, req: Float64RequestRequest):
        # Do not try to create a trajectory unless the robot pose transformation is known
        if self.current_pose is not None:

            # Define the distance to travel and the number of points to generate along the way
            # Also convert the distance between scans to millimeters before using it
            num_points = abs(round(req.value / (self.min_distance_between_registered_scans / 1000)))

            # Save a copy of the robot pose transformation to use to ensure data is not overwritten in the process
            local_pose_transformation = copy(self.current_pose)

            # Calculate the RPY of the current pose to use for the trajectory maintenance
            roll, pitch, yaw = calc_rpy(self.current_pose[0:3, 0:3])

            # Set the current pitch and yaw as the set-points for the angular controllers
            self.set_trajectory_pitch_service(pitch)
            self.set_trajectory_yaw_service(yaw)

            # Create linear vectors on each plane between the current position and a distance in the x-axis containing
            # a given number of points
            trajectory = array([linspace(start=array([0, 0, 0]), stop=array([req.value, 0, 0]), num=num_points),
                                linspace(start=array([0, 1, 0]), stop=array([req.value, 1, 0]), num=num_points),
                                linspace(start=array([0, 0, 1]), stop=array([req.value, 0, 1]), num=num_points)])

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
            self.trajectory = trajectory

            # Set the current set point for the trajectory
            self.update_current_trajectory_set_point()

            # Send the response
            return Float64RequestResponse(was_succesful=True, message=NO_ERROR)

        return Float64RequestResponse(was_succesful=False, message="No known robot pose")

    # Define the service for setting the image spacing
    def set_trajectory_spacing_handler(self, req: Float64RequestRequest):
        self.min_distance_between_registered_scans = req.value
        return Float64RequestResponse(was_succesful=True, message=NO_ERROR)

    # Define service for clearing trajectory
    def clear_trajectory_handler(self, req: BoolRequestRequest):
        if req.value:
            # Clear the trajectory
            self.trajectory = None
            self.current_trajectory_set_point = None
            # Clear the set points in the robot control node
            self.clear_current_set_points_service(True)
        return BoolRequestResponse(was_succesful=True, message=NO_ERROR)

    # Define service for pausing the trajectory
    def pause_trajectory(self, req: BoolRequestRequest):
        self.is_trajectory_paused = req.value
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def update_current_trajectory_set_point(self):
        """
        Updates the current trajectory set point variable and the current trajectory variable depending on the
        level of completion of the trajectory.
        """

        # If the trajectory is not empty
        if self.trajectory is not None and self.trajectory.size > 0:

            # Save the next surface to travel to
            self.current_trajectory_set_point = array([self.trajectory[0][0],
                                                       self.trajectory[1][0],
                                                       self.trajectory[2][0],
                                                       ])

            # Pop the current set point out of the trajectory
            self.trajectory = array([self.trajectory[0][1:],
                                     self.trajectory[1][1:],
                                     self.trajectory[2][1:],
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

            # Update the set point in the robot control node
            self.set_next_waypoint_service(current_trajectory_set_point_message)

        # Otherwise clear the trajectory and the current set point
        else:
            if self.trajectory is not None:
                self.trajectory_complete_service(True)
            self.trajectory = None
            self.current_trajectory_set_point = None
            self.clear_current_set_points_service(True)

    def main_loop(self):

        # if a trajectory exists
        if self.trajectory is not None:
            if self.current_trajectory_set_point is not None:

                # If the state is correct to register data and move on to the next waypoint
                if self.current_pose is not None and self.trajectory is not None and \
                        (self.is_patient_in_contact or self.is_patient_in_contact_override) and \
                        (self.is_proper_force_applied or self.is_proper_force_applied_override) and \
                        (self.is_image_balanced or self.is_image_balanced_override) and \
                        (self.is_image_centered or self.is_image_centered_override) and \
                        not self.is_trajectory_paused and self.trajectory_waypoint_reached:

                    if not self.data_registration_was_requested:

                        # Request to register data
                        resp = self.register_new_data_service(True)

                        self.data_registration_was_requested = resp.was_succesful

                    elif self.data_has_been_registered:
                        # Send the waypoint
                        self.update_current_trajectory_set_point()

                        self.data_has_been_registered = False
                        self.data_registration_was_requested = False


if __name__ == '__main__':

    node = TrajectoryManagementNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
