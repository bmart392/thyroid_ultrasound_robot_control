
# Import standard packages
from numpy import array, append,ndarray
from scipy.spatial.transform import Rotation

# Import ROS packages
from geometry_msgs.msg import Pose


def convert_pose_to_transform_matrix(pose: Pose) -> ndarray:
    """Converts a pose message to a homogeneous transformation matrix."""

    return append(append(Rotation(quat=(pose.orientation.x,
                                        pose.orientation.y,
                                        pose.orientation.z,
                                        pose.orientation.w)).as_matrix(), array([[pose.position.x],
                                                                                 [pose.position.y],
                                                                                 [pose.position.z]]), axis=1),
                  array([[0, 0, 0, 1]]), axis=0)
