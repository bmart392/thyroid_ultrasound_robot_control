#!/usr/bin/env python3

# Import standard ros packages
from rospy import is_shutdown, init_node, Rate, Publisher, Subscriber
from franka_msgs.msg import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, Bool, Float64MultiArray, MultiArrayDimension

# Import from standard packages
from numpy import linspace, array, append, delete
from copy import copy


class TrajectoryGenerationNode:
    def __init__(self):

        # Initialize the node
        init_node('TrajectoryGenerationNode')

        # Define the rate at which trajectory positions will be published
        self.publishing_rate = Rate(60)  # hz

        # Define a variable in which to save the current robot pose transformation
        self.robot_pose_transformation = None

        # Define a variable to save the currently calculated trajectory
        self.current_trajectory = None

        # Define a variable to save the message for the current goal position
        self.current_goal_position_message = None

        # Define a variable to store if the current goal has been reached
        self.current_goal_reached = False

        # Define a variable to store if the positions should be published
        self.publish_trajectory_positions = False

        # Define a

        # Create a subscriber to listen for the current position of the robot
        Subscriber('/franka_state_controller/franka_states', FrankaState, self.robot_current_pose_callback)

        # Create a subscriber to listen for when the current position goal has been reached
        Subscriber('/position_control/goal_reached', Bool, self.goal_reached_callback)

        # Create a subscriber to listen for when the current trajectory should be erased
        Subscriber('/position_control/clear_trajectory', Bool, self.clear_trajectory_callback)

        # Create a subscriber to listen for when to create a trajectory
        Subscriber('/command/create_trajectory', Float64, self.create_trajectory_callback)

        # Create a publisher to publish the next position in the trajectory
        self.trajectory_position_publisher = Publisher('/position_control/goal_surface',
                                                       Float64MultiArray, queue_size=1)

    def create_trajectory_callback(self, data: Float64):

        # Do not try to create a trajectory unless the robot pose transformation is known
        if self.robot_pose_transformation is not None:

            # Define the distance to travel and the number of points to generate along the way
            travel_distance = data.data
            num_points = 10

            # Save a copy of the robot pose transformation to use to ensure data is not overwritten in the process
            local_pose_transformation = copy(self.robot_pose_transformation)

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

            # Generate the new trajectory message to send
            self.generate_trajectory_message()

    def clear_trajectory_callback(self, data: Bool):
        if data.data:
            a = len(self.current_trajectory)
            pass

    def goal_reached_callback(self, data: Bool):

        # If the goal currently published goal has been reached and the current trajectory is not None
        if data.data and self.current_trajectory is not None:

            # Delete the first coordinate in the trajectory
            self.current_trajectory = array([self.current_trajectory[0][1:],
                                             self.current_trajectory[1][1:],
                                             self.current_trajectory[2][1:],
                                             ])

            # If the trajectory is not empty
            if self.current_trajectory.size > 0:

                # Generate the new trajectory message
                self.generate_trajectory_message()

            # Else make both None
            else:
                self.current_trajectory = None
                self.current_goal_position_message = None

    def generate_trajectory_message(self):

        # If the trajectory is not empty
        if self.current_trajectory.size > 0:

            # Save the point to send
            points_to_send = array([self.current_trajectory[0][0],
                                    self.current_trajectory[1][0],
                                    self.current_trajectory[2][0],
                                    ])

            # Create a new multi-dimension array message to transmit this information
            self.current_goal_position_message = Float64MultiArray()

            # Fill in the message with data from the trajectory
            self.current_goal_position_message.layout.dim.append(MultiArrayDimension)
            self.current_goal_position_message.layout.dim[0].label = 'vectors'
            self.current_goal_position_message.layout.dim[0].size = points_to_send.shape[0]
            self.current_goal_position_message.layout.dim[0].stride = points_to_send.size
            self.current_goal_position_message.layout.dim.append(MultiArrayDimension)
            self.current_goal_position_message.layout.dim[1].label = 'dimensions'
            self.current_goal_position_message.layout.dim[1].size = points_to_send.shape[1]
            self.current_goal_position_message.layout.dim[1].stride = points_to_send.shape[1]
            self.current_goal_position_message.data = points_to_send.reshape([points_to_send.size])

    def robot_current_pose_callback(self, data: FrankaState):

        # Save the transformation sent by the robot
        self.robot_pose_transformation = array(data.O_T_EE).reshape((4, 4))

    def main(self):

        # While the node is running
        while not is_shutdown():

            # If there is a goal position message
            if self.current_goal_position_message is not None:

                # Publish it
                self.trajectory_position_publisher.publish(self.current_goal_position_message)

            # Wait to publish the next message
            self.publishing_rate.sleep()


if __name__ == "__main__":

    # Create the node
    node = TrajectoryGenerationNode()

    print("Node initialized. Press ctrl+c to terminate.")

    # Run the main loop until shutdown
    node.main()

    print("Node terminated.")

# Equation of a plane is Ax + By + Cz = 0
# Planes can be defined by 3 points
# Define three arbitrary points on the YZ plane in the end effector plane,
#   use (<goal distance>, 0, 0), (<goal distance>, 1, 0), and (<goal distance>, 0, 1)
# Use the transformation to convert all of them into the origin frame
# Create an equation for that plane
# Calculate the distance in X of the end effector from that plane as the error in the system

# The trajectory generation node interpolates U number of points between the current position and the goal position, W,
#   meters away. All points are calculated in the end effector frame.
# Two additional points are calculated for each point and are used to define the plane for each point.
# All points are transformed into the origin frame and then passed to the Robot Control Node
# The robot control node saves all of the points and then navigates to each one.
# To navigate to a point, the node:
#       Calculates the surface for each waypoint
#       Using the surface, calculates the error between the current robot pose and the surface
#       Moves until the robot reaches the surface
#       Waits until a good picture has been taken
#       Repeats until the goal has been reached OR the thyroid is no longer in the frame
