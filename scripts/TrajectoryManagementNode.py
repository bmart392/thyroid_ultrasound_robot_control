#!/usr/bin/env python3

"""
File containing the TrajectoryManagementNode class.
"""

# Import standard ROS packages
from armer_msgs.msg import ManipulatorState

# Import standard python packages
from numpy import copy, array
from rospy import sleep

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_services.srv import *

# Import custom python packages
from thyroid_ultrasound_robot_control_support.Helpers.convert_pose_to_transform_matrix import \
    convert_pose_to_transform_matrix
from thyroid_ultrasound_support.Constants.SharedConstants import REST_PHASE, GROWTH_PHASE
from thyroid_ultrasound_robot_control_support.Trajectories.SimpleTrajectories.TranslationTrajectory import \
    TranslationTrajectory
from thyroid_ultrasound_robot_control_support.Trajectories import Trajectory


class TrajectoryManagementNode(BasicNode):

    def __init__(self):

        super().__init__()

        # Define variable to store for the current pose of the robot
        self.current_pose = None

        # Define a variable to store the trajectory as a child of the Trajectory class
        self.current_trajectory_object: Trajectory = None

        # Define the default spacing for trajectories
        self.min_distance_between_registered_scans = 0.001  # meters

        # Define flag variables
        self.is_patient_in_contact = False
        self.trajectory_waypoint_reached = False
        self.is_image_centered = False
        self.is_proper_force_applied = False
        self.is_image_balanced = False
        self.is_trajectory_paused = False
        self.data_registration_was_requested = False
        self.data_has_been_registered = False
        self.has_stabilization_been_requested = False
        self.has_segmentation_stabilized = False
        self.complete_trajectory_without_registering_data = False

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

        Subscriber(RC_POSITION_CONTROL_GOAL_REACHED, Bool, self.trajectory_waypoint_reached_callback)
        Subscriber(RC_IMAGE_CONTROL_GOAL_REACHED, Bool, self.is_image_centered_callback)
        Subscriber(RC_FORCE_CONTROL_GOAL_REACHED, Bool, self.is_proper_force_applied_callback)
        Subscriber(RC_IMAGE_BALANCE_GOAL_REACHED, Bool, self.is_image_balanced_callback)

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
        Service(TM_COMPLETE_TRAJECTORY_WITHOUT_DATA, BoolRequest, self.complete_trajectory_without_data)
        Service(TM_DATA_HAS_BEEN_REGISTERED, BoolRequest, self.data_has_been_registered_handler)

        # Define robot control service proxies
        self.clear_current_set_points_service = ServiceProxy(RC_CLEAR_CURRENT_SET_POINTS, BoolRequest)
        self.set_next_feature_waypoint_service = ServiceProxy(RC_SET_NEXT_FEATURE_WAYPOINT, TrajectoryWaypoint)

        # Define real-time segmentation service proxies
        self.set_segmentation_phase_service = ServiceProxy(RTS_SET_SEGMENTATION_PHASE, StringRequest)
        self.has_segmentation_stabilized_service = ServiceProxy(RTS_HAS_SEGMENTATION_STABILIZED, ActionRequest)

        # Define image data registration service proxy
        self.register_new_data_service = ServiceProxy(IPR_REGISTER_NEW_DATA, BoolRequest)

        # Define the user interface proxies
        self.trajectory_complete_service = ServiceProxy(UI_TRAJECTORY_COMPLETE, BoolRequest)

        # Save the current time as the last time an image was published
        self.time_of_last_publishing = Time.now()

        self.log_single_message('Node ready')

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

    # endregion

    # Define service handlers
    # region

    def is_patient_in_contact_override_handler(self, req: BoolRequestRequest):
        self.is_patient_in_contact_override = req.value
        if req.value:
            status_msg = 'active'
        else:
            status_msg = 'inactive'
        self.log_single_message('Patient contact override is ' + status_msg)
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def is_proper_force_applied_override_handler(self, req: BoolRequestRequest):
        self.is_proper_force_applied_override = req.value
        if req.value:
            status_msg = 'active'
        else:
            status_msg = 'inactive'
        self.log_single_message('Proper force applied override is ' + status_msg)
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def is_image_balanced_override_handler(self, req: BoolRequestRequest):
        self.is_image_balanced_override = req.value
        if req.value:
            status_msg = 'active'
        else:
            status_msg = 'inactive'
        self.log_single_message('Proper image balance override is ' + status_msg)
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def is_image_centered_override_handler(self, req: BoolRequestRequest):
        self.is_image_centered_override = req.value
        if req.value:
            status_msg = 'active'
        else:
            status_msg = 'inactive'
        self.log_single_message('Proper image centering override is ' + status_msg)
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def registered_data_success_override_handler(self, req: BoolRequestRequest):
        self.registered_data_success_override = req.value
        if req.value:
            status_msg = 'active'
        else:
            status_msg = 'inactive'
        self.log_single_message('Success of data registration override is ' + status_msg)
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def data_has_been_registered_handler(self, req: BoolRequestRequest):
        self.data_has_been_registered = req.value
        if req.value:
            status_msg = ''
        else:
            status_msg = 'not '
        self.log_single_message('Data has ' + status_msg + 'been registered')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    # endregion

    # Define service for creating trajectory
    def create_trajectory_handler(self, req: Float64RequestRequest):
        # Do not try to create a trajectory unless the robot pose transformation is known
        if self.current_pose is not None:

            # Create the new trajectory object
            self.current_trajectory_object = TranslationTrajectory(
                distance_between_way_points=self.min_distance_between_registered_scans,
                starting_pose=copy(self.current_pose),
                ending_offset_distance=array([req.value, 0, 0]),
                generate_trajectory_on_call=True)

            # Transmit the current waypoint
            self.set_next_feature_waypoint_service(self.current_trajectory_object.get_current().to_msg())

            self.log_single_message('New trajectory created')

            # Send the response
            return Float64RequestResponse(was_successful=True, message=NO_ERROR)

        self.log_single_message('Trajectory could not be created because robot pose was not known')
        return Float64RequestResponse(was_successful=False, message="No known robot pose")

    # Define the service for setting the image spacing
    def set_trajectory_spacing_handler(self, req: Float64RequestRequest):
        self.min_distance_between_registered_scans = req.value
        self.log_single_message('New trajectory spacing set to ' + str(req.value) + ' meters')
        return Float64RequestResponse(was_successful=True, message=NO_ERROR)

    # Define service for clearing trajectory
    def clear_trajectory_handler(self, req: BoolRequestRequest):
        if req.value:
            # Clear the trajectory
            self.current_trajectory_object.clear()

            # Clear the set points in the robot control node
            self.clear_current_set_points_service(True)
            self.log_single_message('Current trajectory cleared')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    # Define the service for completing a trajectory without registering data
    def complete_trajectory_without_data(self, req: BoolRequestRequest):
        self.complete_trajectory_without_registering_data = req.value
        if req.value:
            status_msg = 'without'
        else:
            status_msg = 'with'
        self.log_single_message('Trajectories will be completed ' + status_msg + ' data registration')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    # Define service for pausing the trajectory
    def pause_trajectory(self, req: BoolRequestRequest):
        self.is_trajectory_paused = req.value
        if req.value:
            status_msg = 'paused'
        else:
            status_msg = 'active'
        self.log_single_message('Trajectory progress is ' + status_msg)
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def main_loop(self):

        # Define the default status message
        new_status = None

        # Check that the robot pose is known
        if self.current_pose is not None:

            new_status = NO_TRAJECTORY_EXISTS

            # if a trajectory exists
            if self.current_trajectory_object is not None and not self.current_trajectory_object.is_complete():

                new_status = WAYPOINT_NOT_REACHED

                # If the state is correct to register data and move on to the next waypoint
                if self.current_pose is not None and \
                        (self.is_patient_in_contact or self.is_patient_in_contact_override) and \
                        (self.is_proper_force_applied or self.is_proper_force_applied_override) and \
                        (self.is_image_balanced or self.is_image_balanced_override) and \
                        (self.is_image_centered or self.is_image_centered_override) and \
                        not self.is_trajectory_paused and self.trajectory_waypoint_reached:

                    new_status = WAYPOINT_REACHED

                    # If the segmentation has not been requested to stabilize
                    if not self.has_stabilization_been_requested:

                        self.log_single_message('Current waypoint reached')

                        # Request the segmentation stabilize
                        resp = self.set_segmentation_phase_service(REST_PHASE)

                        # Save the status of the request
                        self.has_stabilization_been_requested = resp.was_successful

                        self.log_single_message('Segmentation requested to stabilize')

                    # If the segmentation has not stabilized
                    elif not self.has_segmentation_stabilized:

                        new_status = WAITING_FOR_SEGMENTATION_STABILIZATION

                        # Request an update on the status of the stabilization
                        resp = self.has_segmentation_stabilized_service()

                        # Save the status of the request
                        self.has_segmentation_stabilized = resp.was_successful

                        # If it has not stabilized
                        if not self.has_segmentation_stabilized:
                            # Sleep for 0.25 seconds
                            sleep(0.25)

                    elif self.complete_trajectory_without_registering_data:

                        self.log_single_message('Segmentation has stabilized')

                        # Skip the request to register data step
                        self.data_registration_was_requested = True

                        # Override that data has been registered
                        self.data_has_been_registered = True

                        self.log_single_message('Data will not be registered for this waypoint')

                    elif not self.data_registration_was_requested:

                        self.log_single_message('Segmentation has stabilized')

                        # Request to register data
                        resp = self.register_new_data_service(True)

                        self.data_registration_was_requested = resp.was_successful

                        self.log_single_message('Data was requested to be registered')

                    elif self.data_has_been_registered:

                        # Update the trajectory
                        self.current_trajectory_object.update()

                        # Send the next waypoint to the robot
                        if self.current_trajectory_object.is_complete():
                            self.clear_current_set_points_service(True)
                        else:
                            self.set_next_feature_waypoint_service(
                                self.current_trajectory_object.get_current().to_msg())

                        # Set the segmentation back to growth mode
                        self.set_segmentation_phase_service(GROWTH_PHASE)

                        # Reset the other flags
                        self.has_segmentation_stabilized = False
                        self.has_stabilization_been_requested = False
                        self.data_has_been_registered = False
                        self.data_registration_was_requested = False

                        self.log_single_message('A new waypoint has been set')

        self.publish_node_status(new_status=new_status, delay_publishing=0.5, default_status=ROBOT_POSE_UNKNOWN)


if __name__ == '__main__':

    node = TrajectoryManagementNode()

    rate = Rate(150)

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        node.main_loop()
        rate.sleep()

    print("Node terminated.")
