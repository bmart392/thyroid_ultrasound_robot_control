from TrajectoryManagementNode import TrajectoryManagementNode
from armer_msgs.msg import ManipulatorState
from thyroid_ultrasound_services.srv import Float64RequestRequest, BoolRequestRequest

# Create the node to test
validation_node = TrajectoryManagementNode()

# Send a pose to the node as the current pose of the robot
current_pose_message = ManipulatorState()
current_pose_message.ee_pose.pose.position.x = 0
current_pose_message.ee_pose.pose.position.y = 0
current_pose_message.ee_pose.pose.position.z = 0
current_pose_message.ee_pose.pose.orientation.x = 1
current_pose_message.ee_pose.pose.orientation.y = 0
current_pose_message.ee_pose.pose.orientation.z = 0
current_pose_message.ee_pose.pose.orientation.w = 0
validation_node.current_pose_callback(current_pose_message)

# Set the others variable that enable the trajectory to continue
validation_node.is_patient_in_contact_override_handler(BoolRequestRequest(True))
validation_node.is_proper_force_applied_override_handler(BoolRequestRequest(True))
validation_node.is_image_balanced_override_handler(BoolRequestRequest(True))
validation_node.is_image_centered_override_handler(BoolRequestRequest(True))

# Set the trajectory spacing
validation_node.set_trajectory_spacing_handler(Float64RequestRequest(0.0005))

# Request to create a trajectory
request = Float64RequestRequest(0.01)
validation_node.create_trajectory_handler(request)

# Clear the trajectory
validation_node.clear_trajectory_handler(BoolRequestRequest(True))

# Build a new trajectory
validation_node.create_trajectory_handler(Float64RequestRequest(0.003))

while len(validation_node.current_trajectory_object.components_remaining) > 0:
    validation_node.current_trajectory_object.update()

print('Done')
