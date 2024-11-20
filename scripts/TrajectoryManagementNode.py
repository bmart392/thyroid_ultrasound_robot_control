#!/usr/bin/env python3

"""
File containing the TrajectoryManagementNode class.
"""

# Import standard ROS packages
from armer_msgs.msg import ManipulatorState

# Import standard python packages
from rospy import sleep
from thyroid_ultrasound_robot_control_support.Trajectories.SimpleTrajectories.PoseToPoseTrajectory import \
    PoseToPoseTrajectory

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
from thyroid_ultrasound_robot_control_support.Trajectories.TrajectoryAction import *
from thyroid_ultrasound_robot_control_support.TracjectoryManagementNodeConstants import *
from thyroid_ultrasound_support.MessageConversion.convert_float64_multi_array_message_to_array import \
    convert_float64_multi_array_message_to_array


class TrajectoryManagementNode(BasicNode):

    def __init__(self):

        super().__init__()

        # Define variables to track current information about the system
        self.current_action: str = WAITING_FOR_ACTION
        # noinspection PyTypeChecker
        self.current_scan_type: str = None
        # noinspection PyTypeChecker
        self.current_interrupt_type: str = None
        self.current_pose = None
        self.current_trajectory_object: Trajectory = None
        self.current_scan_distance: array = None
        self.current_distance_registered_data = 0.001  # meters
        self.trajectory_action_queue = []

        # Define a variable to save the mask corresponding to the recovery pose
        self.mask_at_recovery_pose_as_msg: array = None

        # Define flag variables
        self.is_roi_shown = False
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
        self.reset_image_mask_at_end_of_trajectory = False

        # Define user override flag variables
        self.is_patient_in_contact_user_override = False
        self.is_proper_force_applied_user_override = False
        self.is_image_balanced_user_override = False
        self.is_image_centered_user_override = False
        self.registered_data_success_user_override = False

        # Define trajectory override flag variables
        self.is_patient_in_contact_trajectory_override = False
        self.is_proper_force_applied_trajectory_override = False
        self.is_image_balanced_trajectory_override = False
        self.is_image_centered_trajectory_override = False
        self.registered_data_success_trajectory_override = False
        self.complete_trajectory_without_registering_data_trajectory_override = False

        # Initialize the ROS node
        init_node(TRAJECTORY_MANAGEMENT)

        # Define robot pose subscriber
        Subscriber(ARMER_STATE, ManipulatorState, self.current_pose_callback)

        # Define status subscribers
        Subscriber(IMAGE_PATIENT_CONTACT, Bool, self.is_patient_in_contact_callback)
        Subscriber(IMAGE_ROI_SHOWN, Bool, self.is_roi_shown_callback)

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
        # Service(TM_CREATE_TRAJECTORY, Float64Request, self.create_trajectory_handler)
        Service(TM_INITIATE_SCAN, InitiateScan, self.initiate_scan_handler)
        Service(TM_SET_TRAJECTORY_SPACING, Float64Request, self.set_trajectory_spacing_handler)
        Service(TM_INTERRUPT_TRAJECTORY, StringRequest, self.trajectory_interruption_handler)
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
        self.retrieve_last_valid_data_service = ServiceProxy(IPR_RETRIEVE_VALID_DATA, ValidData)
        self.save_valid_positions_service = ServiceProxy(IPR_SAVE_VALID_POSITIONS, SaveValidPositionsSettings)

        # Define the user interface proxies
        self.scanning_complete_service = ServiceProxy(UI_SCANNING_COMPLETE, BoolRequest)

        # Define the service proxy for resetting the image segmentation mask
        self.reset_image_mask_service = ServiceProxy(RTS_UPDATE_INITIALIZATION_MASK, UpdateInitializationMask)

        # Save the current time as the last time an image was published
        self.time_of_last_publishing = Time.now()

        self.log_single_message('Node ready')

    # Define status subscribers
    # region

    def current_pose_callback(self, msg: ManipulatorState):
        self.current_pose = convert_pose_to_transform_matrix(msg.ee_pose.pose)

    def is_roi_shown_callback(self, msg: Bool):
        self.is_roi_shown = msg.data

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
        """Saves the override signal received by the node and logs the value of the signal received"""
        self.is_patient_in_contact_user_override = req.value
        self.log_status_signal(req.value, 'Patient contact override is')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def is_proper_force_applied_override_handler(self, req: BoolRequestRequest):
        """Saves the override signal received by the node and logs the value of the signal received."""
        self.is_proper_force_applied_user_override = req.value
        self.log_status_signal(req.value, 'Proper force applied override is')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def is_image_balanced_override_handler(self, req: BoolRequestRequest):
        """Saves the override signal received by the node and logs the value of the signal received."""
        self.is_image_balanced_user_override = req.value
        self.log_status_signal(req.value, 'Proper image balance override is')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def is_image_centered_override_handler(self, req: BoolRequestRequest):
        """Saves the override signal received by the node and logs the value of the signal received."""
        self.is_image_centered_user_override = req.value
        self.log_status_signal(req.value, 'Proper image centering override is')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def registered_data_success_override_handler(self, req: BoolRequestRequest):
        """Saves the override signal received by the node and logs the value of the signal received."""
        self.registered_data_success_user_override = req.value
        self.log_status_signal(req.value, 'Success of data registration override is')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def data_has_been_registered_handler(self, req: BoolRequestRequest):
        """Saves the status signal received by the node and logs the value of the signal received."""
        self.data_has_been_registered = req.value
        self.log_status_signal(req.value, 'Data has', suffix_text='been registered',
                               true_option='', false_option='not')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def complete_trajectory_without_data(self, req: BoolRequestRequest):
        """Saves the override signal received by the node and logs the value of the signal received."""
        self.complete_trajectory_without_registering_data = req.value
        self.log_status_signal(req.value, 'Trajectories will be completed', suffix_text='data registration',
                               true_option='without', false_option='with')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    def pause_trajectory(self, req: BoolRequestRequest):
        """Saves the override signal received by the node and logs the value of the signal received."""
        self.is_trajectory_paused = req.value
        self.log_status_signal(req.value, 'Trajectory progress is', true_option='paused', false_option='active')
        return BoolRequestResponse(was_successful=True, message=NO_ERROR)

    # endregion

    # Define trajectory management service handlers
    # region

    def initiate_scan_handler(self, req: InitiateScanRequest):
        """Initiates a scan of the requested type, using the given offset if provided"""
        # Do not try to create a scan unless the robot pose transformation is known
        if self.current_pose is not None:

            # If a single scan is being completed, set the scanning distance
            if req.scan_type == SINGLE_DIRECTION:
                if req.optional_scan_axis in (X_AXIS_OFFSET, Y_AXIS_OFFSET, Z_AXIS_OFFSET):
                    local_scan_distance = [0, 0, 0]
                    local_scan_distance[req.optional_scan_axis] = req.optional_scan_distance
                    self.current_scan_distance = array(local_scan_distance)
                else:
                    return InitiateScanResponse(was_successful=False, message='Scan axis was not recognized')

            # If a bidirectional or dual-lobe scan is being completed, do nothing
            elif req.scan_type == BI_DIRECTION or req.scan_type == DUAL_LOBE_SCAN:
                pass

            # Otherwise return a failure response
            else:
                return InitiateScanResponse(was_successful=False, message='Scan type was not recognized')

            # Set the scan type and current action accordingly
            self.current_scan_type = req.scan_type
            self.current_action = GENERATING_TRAJECTORY_OBJECTS

            # Log the success
            self.log_single_message('New scan added')

            # Send the response
            return InitiateScanResponse(was_successful=True, message=NO_ERROR)

        # Otherwise log the result and send the response
        self.log_single_message('Scan could not be created because robot pose was not known')
        return InitiateScanResponse(was_successful=False, message="No known robot pose")

    def set_trajectory_spacing_handler(self, req: Float64RequestRequest):
        """Sets the spacing between waypoints on the trajectory in meters"""
        if self.current_scan_type is None:
            self.current_distance_registered_data = req.value
            self.log_single_message('New trajectory spacing set to ' + str(req.value) + ' meters')
            return Float64RequestResponse(was_successful=True, message=NO_ERROR)
        return Float64RequestResponse(was_successful=False, message='Could not change spacing while scan is active')

    # Define service for clearing trajectory
    def trajectory_interruption_handler(self, req: StringRequestRequest):
        """Interrupts the current trajectory or stops the whole scan depending on the request received"""
        if req.value in (FULL_STOP, MOVE_TO_NEXT_TRAJECTORY_SEGMENT):
            self.current_interrupt_type = req.value
            self.current_action = INTERRUPT_CURRENT_TRAJECTORY
            self.log_single_message('Received signal to: ' + req.value)
            return StringRequestResponse(was_successful=True, message=NO_ERROR)
        return StringRequestResponse(was_successful=False, message='Type of interrupt requested was not recognized')

    # endregion

    # Define helper functions
    # region

    def log_status_signal(self, decision_value: bool, prefix_text: str, suffix_text: str = '',
                          true_option: str = 'active', false_option: str = 'inactive'):
        """
        Wraps the log_single_message function to send one of two text options based on the given value.

        Parameters
        ----------
        decision_value
            The value that determine whether the 'true_option' text or the 'false_option' text is used
        prefix_text
            The text placed before the 'true_option' of 'false_option' text
        suffix_text
            The text placed after the 'true_option' of 'false_option' text
        true_option
            The text inserted if the decision value is true
        false_option
            The text inserted if the decision value is false
        """
        # Select the correct status based on the decision value
        if decision_value:
            status_msg = true_option
        else:
            status_msg = false_option

        # Add a space to the suffix text if text is given
        if len(suffix_text) > 0:
            suffix_text = ' ' + suffix_text

        # Log the full message
        self.log_single_message(prefix_text + ' ' + status_msg + suffix_text)

    def clear_current_trajectory(self):
        """Clears the current trajectory."""
        # Clear the current trajectory
        self.current_trajectory_object.clear()

        # Reset all the trajectory overrides
        self.set_all_trajectory_overrides(False)

        # Reset information about saving mask at end of trajectory
        self.reset_image_mask_at_end_of_trajectory = False
        self.mask_at_recovery_pose_as_msg = None

        # Clear the set points from the robot control node
        self.clear_current_set_points_service(True)

    def clear_current_scan(self):
        """Clears the current scan."""

        # Clear the current trajectory
        self.clear_current_trajectory()

        # Update the UI to indicate that scanning is complete
        self.scanning_complete_service(True)

        # Clear the current scan type
        self.current_scan_type = None

        # Clear the current interruption type
        self.current_interrupt_type = None

        # Clear the current scan distance
        self.current_scan_distance = None

        # Set the current action variable for the next state
        self.current_action = WAITING_FOR_ACTION

    def set_all_trajectory_overrides(self, set_overrides: bool):
        """Applies all trajectory-based overrides"""
        self.complete_trajectory_without_registering_data_trajectory_override = set_overrides
        self.is_patient_in_contact_trajectory_override = set_overrides
        self.is_proper_force_applied_trajectory_override = set_overrides
        self.is_image_balanced_trajectory_override = set_overrides
        self.is_image_centered_trajectory_override = set_overrides

    def set_overrides_for_translation_trajectory(self, set_overrides: bool):
        """Applies all overrides for translation trajectory"""
        self.complete_trajectory_without_registering_data_trajectory_override = set_overrides

    # endregion

    def main_loop(self):

        # Define the default status message
        new_status = None

        # Check that the robot pose is known
        if self.current_pose is not None:

            # Do nothing if there are no actions to complete
            if self.current_action == WAITING_FOR_ACTION:
                new_status = WAITING

            elif self.current_action == GENERATING_TRAJECTORY_OBJECTS:
                if self.current_scan_type == SINGLE_DIRECTION:

                    # Create a trajectory action object for a surface trajectory and add it to the queue
                    self.trajectory_action_queue.append(
                        TrajectoryAction(trajectory_type=SURFACE_BASED,
                                         waypoint_spacing=self.current_distance_registered_data,
                                         ending_offset=self.current_scan_distance,
                                         starting_pose_src=CURRENT_ROBOT_POSE)
                    )

                elif self.current_scan_type == BI_DIRECTION:

                    # Create a trajectory action object for the first surface trajectory
                    self.trajectory_action_queue.append(
                        TrajectoryAction(trajectory_type=SURFACE_BASED,
                                         waypoint_spacing=self.current_distance_registered_data,
                                         ending_offset=array([0.06, 0, 0]),
                                         starting_pose_src=CURRENT_ROBOT_POSE,
                                         include_standard_overrides=True
                                         )
                    )

                    # Create a trajectory action object for a pose-to-pose trajectory
                    # between the end of the last trajectory and the last valid pose
                    self.trajectory_action_queue.append(
                        TrajectoryAction(trajectory_type=POSE_TO_POSE,
                                         waypoint_spacing=NO_WAYPOINTS,
                                         starting_pose_src=CURRENT_ROBOT_POSE,
                                         ending_pose_src=LAST_VALID_POSE,
                                         reset_image_mask_at_end=True,
                                         include_standard_overrides=True
                                         )
                    )

                    # Create a trajectory action object for a surface trajectory
                    # using the starting pose, a large offset, and the opposite direction
                    # from whatever was given before
                    self.trajectory_action_queue.append(
                        TrajectoryAction(trajectory_type=SURFACE_BASED,
                                         waypoint_spacing=self.current_distance_registered_data,
                                         ending_offset=array([-0.10, 0, 0]),
                                         starting_pose_src=CURRENT_ROBOT_POSE,
                                         )
                    )

                elif self.current_scan_type == DUAL_LOBE_SCAN:
                    # This is not going to be implemented
                    self.clear_current_scan()

                # Set the current action variable for the next state
                self.current_action = PREPARING_TO_NAVIGATE_CURRENT_TRAJECTORY

            elif self.current_action == PREPARING_TO_NAVIGATE_CURRENT_TRAJECTORY:

                # If there is another trajectory action to complete
                if len(self.trajectory_action_queue) > 0:

                    # Pop out the next trajectory action from the queue
                    current_trajectory_action: TrajectoryAction = self.trajectory_action_queue.pop(0)

                    # Capture the starting pose, if necessary
                    if current_trajectory_action.starting_pose is not None:
                        this_starting_pose = current_trajectory_action.starting_pose
                    elif current_trajectory_action.starting_pose_src is not None:
                        this_starting_pose = self.current_pose
                    else:
                        raise Exception("Starting pose and starting pose source cannot be None.")

                    # Create the correct trajectory object from the action object
                    if current_trajectory_action.trajectory_type == SURFACE_BASED:
                        new_trajectory = TranslationTrajectory(
                            distance_between_way_points=current_trajectory_action.waypoint_spacing,
                            starting_pose=this_starting_pose,
                            ending_offset_distance=current_trajectory_action.ending_offset,
                            generate_trajectory_on_call=True)

                        if current_trajectory_action.include_standard_overrides:
                            self.set_overrides_for_translation_trajectory(True)

                    elif current_trajectory_action.trajectory_type == POSE_TO_POSE:

                        # Capture the ending pose
                        if current_trajectory_action.ending_pose is not None:
                            this_ending_pose = current_trajectory_action.ending_pose
                        elif current_trajectory_action.ending_pose_src is not None:
                            resp: ValidDataResponse = self.retrieve_last_valid_data_service(10)
                            this_ending_pose = convert_float64_multi_array_message_to_array(resp.valid_pose)

                            # Save whether the image mask should be reset at the end of the trajectory and the
                            # mask itself if necessary
                            if current_trajectory_action.reset_image_mask_at_end:
                                self.reset_image_mask_at_end_of_trajectory = \
                                    current_trajectory_action.reset_image_mask_at_end
                                self.mask_at_recovery_pose_as_msg = resp.valid_mask

                        else:
                            raise Exception("Starting pose and starting pose source cannot be None.")

                        new_trajectory = PoseToPoseTrajectory(
                            distance_between_way_points=current_trajectory_action.waypoint_spacing,
                            starting_pose=this_starting_pose,
                            ending_pose=this_ending_pose,
                            generate_trajectory_on_call=True)

                        # Set the overrides for the trajectory
                        if current_trajectory_action.include_standard_overrides:
                            self.set_all_trajectory_overrides(True)

                    else:
                        raise Exception("Trajectory type of '" + current_trajectory_action.trajectory_type +
                                        "' is not recognized.")

                    # Set the current trajectory object to be the first one in the list of trajectory actions
                    self.current_trajectory_object = new_trajectory

                    # Send the appropriate set point to the robot control node
                    self.set_next_feature_waypoint_service(self.current_trajectory_object.get_current().to_msg())

                    # Set the Image Position Registration Node to start saving valid positions
                    self.save_valid_positions_service(.1, 3)

                    # Set the current action variable for the next state
                    self.current_action = NAVIGATING_CURRENT_TRAJECTORY

                # Otherwise, interrupt the scan and come to a full stop
                else:
                    self.current_action = INTERRUPT_CURRENT_TRAJECTORY
                    self.current_interrupt_type = FULL_STOP

            elif self.current_action == NAVIGATING_CURRENT_TRAJECTORY:

                new_status = NO_TRAJECTORY_EXISTS

                # if a trajectory exists
                if self.current_trajectory_object is not None and not self.current_trajectory_object.is_complete():

                    new_status = WAYPOINT_NOT_REACHED

                    # If the state is correct to register data and move on to the next waypoint
                    if (self.is_patient_in_contact or self.is_patient_in_contact_user_override
                        or self.is_patient_in_contact_trajectory_override) and \
                            (self.is_proper_force_applied or self.is_proper_force_applied_user_override
                             or self.is_proper_force_applied_trajectory_override) and \
                            (self.is_image_balanced or self.is_image_balanced_user_override
                             or self.is_image_balanced_trajectory_override) and \
                            (self.is_image_centered or self.is_image_centered_user_override
                             or self.is_image_centered_trajectory_override) and \
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

                        elif self.complete_trajectory_without_registering_data \
                                or self.complete_trajectory_without_registering_data_trajectory_override:

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

                            # If the current trajectory is complete, change the current action
                            if self.current_trajectory_object.is_complete():
                                self.current_action = CLOSING_CURRENT_TRAJECTORY

                            # Otherwise, send the next waypoint in the trajectory
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

                    # If the ROI has been lost in the image, stop the current trajectory
                    elif not self.is_roi_shown:
                        self.current_action = CLOSING_CURRENT_TRAJECTORY

            elif self.current_action == CLOSING_CURRENT_TRAJECTORY:

                # Reset the image segmentation mask, if needed
                if self.reset_image_mask_at_end_of_trajectory:
                    self.reset_image_mask_service(None, self.mask_at_recovery_pose_as_msg)

                # Tell the Image Position Registration Node to stop saving valid positions
                self.save_valid_positions_service(0, 0)

                # If completing a single direction scan, use the interrupt to come to a full stop
                if self.current_scan_type == SINGLE_DIRECTION:
                    self.current_action = INTERRUPT_CURRENT_TRAJECTORY
                    self.current_interrupt_type = FULL_STOP

                # If completing a bidirectional scan, use the interrupt to complete only the current segment
                elif self.current_scan_type == BI_DIRECTION:
                    self.current_action = INTERRUPT_CURRENT_TRAJECTORY
                    self.current_interrupt_type = MOVE_TO_NEXT_TRAJECTORY_SEGMENT

                elif self.current_scan_type == DUAL_LOBE_SCAN:
                    pass

            elif self.current_action == INTERRUPT_CURRENT_TRAJECTORY:

                if self.current_interrupt_type == FULL_STOP:
                    self.clear_current_scan()

                elif self.current_interrupt_type == MOVE_TO_NEXT_TRAJECTORY_SEGMENT:
                    self.clear_current_trajectory()
                    self.current_action = PREPARING_TO_NAVIGATE_CURRENT_TRAJECTORY

                else:
                    raise Exception('Interrupt type of "' + self.current_interrupt_type + '" not recognized')

                # Clear the interrupt type
                self.current_interrupt_type = None

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
