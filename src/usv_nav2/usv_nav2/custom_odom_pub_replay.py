import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
import math
import tf_transformations

import re

pattern = r'(\d+)\s+\((\d+),(\d+)\)\s+([\d.]+)'

pose_list = []

class CustomOdometryPublisher(Node):
    def __init__(self):
        super().__init__('custom_odometry_publisher')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.2, self.publish_odometry)

        self.pose_index = 0


    def publish_odometry(self):
        now = self.get_clock().now().to_msg()


        #USE POSE TXT, THIS WILL NEED TO BE UPDATED TO ACCEPT LIVE FEED

        if(self.pose_index >= len(pose_list)):
            self.pose_index = 0

        x = pose_list[self.pose_index][1]
        y = pose_list[self.pose_index][2]
        theta = pose_list[self.pose_index][3] * (-math.pi / 180.0) - math.pi / 2.0 #negate radians and rotate 90 degrees
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.pose_index += 1

        # 1. Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        # (optional) set velocity or covariance if needed
        self.odom_pub.publish(odom)

def main(args=None):

    global pose_list

    with open('/home/ws/map/demo/7_pose_estimation.txt', 'r') as file:
        content = file.read()

    # Apply regex to extract data
    matches = re.findall(pattern, content)

    # Convert string matches to proper types
    pose_list = [(int(a), (float(b)-250.0)*0.05, (float(c)-250.0)*-0.05, float(d)) for a, b, c, d in matches]

    rclpy.init(args=args)
    node = CustomOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
