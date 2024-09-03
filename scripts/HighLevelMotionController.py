#!/usr/bin/env python3

"""
File containing the HighLevelMotionController class.
"""

# TODO - Make the trajectory accept a pose destination
# TODO - Make the segmentation be able to initialize off of a down-sampled ground truth mask

# Import standard ros packages


# Import standard packages
from rospy import sleep

# Import custom ROS packages

# Import custom python packages
from thyroid_ultrasound_support.BasicNode import *

# Define the actions that the node can be completed
SLSD_SCAN: str = 'single lobe single direction scan'
SLBD_SCAN: str = 'single lobe bi-direction scan'
DLSD_SCAN: str = 'double lobe bi-direction scan'
DLBD_SCAN: str = 'double lobe bi-direction scan'
NO_ACTION: str = 'no action'

# Define the allowed states of the state machine
READY_TO_START_ACTION: str = 'ready to start action'
WAITING_FOR_EXTERNAL_STIMULUS: str = 'waiting for external stimulus'
READY_TO_CLOSE_CURRENT_ACTION: str = 'ready to close current action'


class HighLevelMotionController(BasicNode):

    def __init__(self):

        # Call the super class
        super().__init__()

        self.actions_to_complete = []

        self.current_action = None

        self.current_state = None

        self.segmentation_running = False

        self.trajectory_completed = False

        self.first_trajectory_completed = False
        self.second_trajectory_started = False

        self.last_valid_mask = None

        self.scan_distances = []
        self.current_scan_distances = None

        # Define services
        Service(HLMC_SCAN_SINGLE_LOBE, SingleLobeScan, self.scan_single_lobe_handler)

        # Define the service proxies
        self.create_trajectory_service = ServiceProxy(TM_CREATE_TRAJECTORY, Float64Request)
        self.complete_trajectory_without_registering_data_service = ServiceProxy(TM_COMPLETE_TRAJECTORY_WITHOUT_DATA,
                                                                                 BoolRequest)
        self.save_valid_positions_service = ServiceProxy(IPR_SAVE_VALID_POSITIONS, Float64Request)
        self.user_finish_scan_service = ServiceProxy(UI_USER_FINISH_SCAN, ActionRequest)

        self.retrieve_valid_data_service = ServiceProxy(IPR_RETRIEVE_VALID_DATA, ValidData)

        self.move_to_pose_service = ServiceProxy(TM_CREATE_POSE_TRAJECTORY, Float64MultiArrayRequest)

    def node_status_reader(self, status_msg: log_message):

        if status_msg.source == REAL_TIME_IMAGE_FILTER:
            self.segmentation_running = status_msg.message == ANALYZING_IMAGE

        elif status_msg.source == TRAJECTORY_MANAGEMENT:
            self.trajectory_completed = status_msg.message == NO_TRAJECTORY_EXISTS

    def scan_single_lobe_handler(self, req: SingleLobeScanRequest):

        if req.action == SLSD_SCAN:
            self.actions_to_complete.append(SLSD_SCAN)
            self.scan_distances.append([req.direction_1_distance])
        elif req.action == SLBD_SCAN:
            self.actions_to_complete.append(SLBD_SCAN)
            self.scan_distances.append([req.direction_1_distance, req.direction_2_distance])
        else:
            return SingleLobeScanResponse(was_successful=False, message="Action type of '" + str(req.action) +
                                                                       "' was not recognized.")

    def main(self):

        # Check the current state
        if self.current_state == READY_TO_START_ACTION:

            # Do nothing unless actions have been commanded
            if len(self.actions_to_complete) > 0:

                # Pull out the oldest action
                self.current_action = self.actions_to_complete.pop(0)

                # Pull out the corresponding distances
                self.current_scan_distances = self.scan_distances.pop(0)

                if self.current_action == SLSD_SCAN:

                    # Send command to start conducting a trajectory
                    self.create_trajectory_service(self.current_scan_distances[0])

                elif self.current_action == SLBD_SCAN:
                    # Send command to trajectory management node to not ask to register data
                    self.complete_trajectory_without_registering_data_service(True)

                    # Send command to start conducting a trajectory
                    self.create_trajectory_service(self.current_scan_distances[0])

                    # Send command to start saving a position and image mask every 0.25 seconds
                    self.save_valid_positions_service(0.25)

                    pass

                elif self.current_action == DLSD_SCAN:
                    pass
                elif self.current_action == DLBD_SCAN:
                    pass
                else:
                    raise Exception("'" + self.current_action + "'" + ' is not a recognized action.')

                # Set the next state
                self.current_state = WAITING_FOR_EXTERNAL_STIMULUS
                return

        elif self.current_state == WAITING_FOR_EXTERNAL_STIMULUS:

            if self.current_action == SLSD_SCAN:

                if self.segmentation_running:
                    return

                elif self.trajectory_completed:
                    self.current_state = READY_TO_CLOSE_CURRENT_ACTION
                    return

                else:

                    # Ask the user to either reset the segmentation or end the trajectory
                    self.user_finish_scan_service()
                    return

            elif self.current_action == SLBD_SCAN:

                if not self.segmentation_running and \
                        not self.trajectory_completed and \
                        not self.first_trajectory_completed:

                    # Note that the first trajectory has finished
                    self.first_trajectory_completed = True

                    # Recall the last position before failure and the corresponding image mask
                    response: ValidDataResponse = self.retrieve_valid_data_service(0)

                    self.last_valid_mask = response.valid_mask

                    # Command the robot to return to the last known position
                    self.move_to_pose_service(response.valid_pose)

                    return

                # If the robot has returned to the last known position,
                elif self.first_trajectory_completed and \
                        self.trajectory_completed and \
                        not self.second_trajectory_started:

                    self.second_trajectory_started = True

                    # Re-initialize the segmentation
                    self.initialize_image_filter_from_previous_mask(self.last_valid_mask)

                    # Send the command to start conducting a trajectory
                    self.create_trajectory_service(self.current_scan_distances[1])

                    return

                # If the trajectory fails or completes,
                elif self.first_trajectory_completed and \
                        self.second_trajectory_started and \
                        not self.segmentation_running and \
                        not self.trajectory_completed:
                    self.current_state = READY_TO_CLOSE_CURRENT_ACTION
                    return
                pass
            elif self.current_action == DLSD_SCAN:
                pass
            elif self.current_action == DLBD_SCAN:
                pass
            else:
                raise Exception("'" + self.current_action + "'" + ' is not a recognized action.')

        elif self.current_state == READY_TO_CLOSE_CURRENT_ACTION:

            self.first_trajectory_completed = False
            self.second_trajectory_started = False

            self.current_scan_distances = None
            self.current_action = NO_ACTION
            self.current_state = READY_TO_START_ACTION


if __name__ == '__main__':

    # Create the node
    node = HighLevelMotionController()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        # Run the main function of the node
        node.main()
