from TrajectoryManagementNode import TrajectoryManagementNode
from armer_msgs.msg import ManipulatorState
from thyroid_ultrasound_services.srv import Float64RequestRequest

# Create the node to test
validation_node = TrajectoryManagementNode()

# Send a pose to the node
current_pose_message = ManipulatorState()
current_pose_message.ee_pose.pose.position.x = 0
current_pose_message.ee_pose.pose.position.y = 0
current_pose_message.ee_pose.pose.position.z = 0
current_pose_message.ee_pose.pose.orientation.x = 1
current_pose_message.ee_pose.pose.orientation.y = 0
current_pose_message.ee_pose.pose.orientation.z = 0
current_pose_message.ee_pose.pose.orientation.w = 0
validation_node.current_pose_callback(current_pose_message)

# Request to create a node
validation_node.create_trajectory_handler(Float64RequestRequest(0.01))

while validation_node.trajectory is not None:
    validation_node.update_current_trajectory_set_point()

print('Done')
