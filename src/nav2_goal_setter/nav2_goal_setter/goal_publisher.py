import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import math
import tf_transformations
import re

pattern = r'(\d+)\s+\((\d+),(\d+)\)\s+([\d.]+)'

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.get_logger().info('Goal Publisher has started.')

    def publish_goal(self):
        with open('/home/ws/map/7_pose_estimation.txt', 'r') as file:
            content = file.read()

        matches = re.findall(pattern, content)
        pose_list = [(int(a), (float(b)-250.0)*0.05, (float(c)-250.0)*-0.05, float(d)) for a, b, c, d in matches]
        x = pose_list[8][1]
        y = pose_list[8][2]
        theta = pose_list[8][3] * (-math.pi / 180.0) - math.pi / 2.0 #negate radians and rotate 90 degrees
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'  # Change the frame_id to the frame you are working in (e.g., "map", "odom")

        # Set the position and orientation of the goal
        goal.pose.position = Point(x=x, y=y, z=0.0)  # Specify your goal position (in meters)
        goal.pose.orientation = quat  # Specify your goal orientation (no rotation)

        # Publish the goal
        self.publisher_.publish(goal)
        self.get_logger().info(f'Publishing goal: {goal.pose.position.x}, {goal.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

