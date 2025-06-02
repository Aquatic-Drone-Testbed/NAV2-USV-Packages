import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
import math
import tf_transformations
from rclpy.callback_groups import ReentrantCallbackGroup
import os



import re

#pattern = r'(\d+)\s+\((\d+),(\d+)\)\s+([\d.]+)'
pattern = r'(\d+)\t\((\d+),(\d+)\)\t(-?\d+\.\d+)'

class CustomOdometryPublisher(Node):
    def __init__(self):
        super().__init__('custom_odometry_publisher')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        reentrant_callback_group = ReentrantCallbackGroup() 

        self.timer = self.create_timer(0.2, self.publish_odometry, reentrant_callback_group)

        self.pose_index = 0
        self.pose_list = []

   

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()

        # print("LOOP")

        file_path = '/home/ws/map/current_pose_estimation.txt'

        if not os.path.exists(file_path):
            return

        # with open(file_path, 'r') as file:
        #     content = file.read()

        file = open(file_path, 'r')
        content = file.read()
        #print(content)
        #file.close()

        # Apply regex to extract data

        matches = re.findall(pattern, content)
        # print(matches)
        # Convert string matches to proper types
        temp_list = [(int(a), (float(b)-250.0)*0.05, (float(c)-250.0)*-0.05, float(d)) for a, b, c, d in matches]
        # print(temp_list)

        if not temp_list and not self.pose_list:
            #print("Empty")
            return
        

        if temp_list:
            self.pose_list = temp_list

        x = self.pose_list[self.pose_index][1]
        y = self.pose_list[self.pose_index][2]
        theta = self.pose_list[self.pose_index][3] * (math.pi / 180.0) + math.pi / 2.0 #negate radians and rotate 90 degrees
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

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

    rclpy.init(args=args)
    node = CustomOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
