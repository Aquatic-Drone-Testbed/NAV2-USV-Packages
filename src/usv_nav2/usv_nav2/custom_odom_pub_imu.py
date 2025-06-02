import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
import tf_transformations
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import math
import socket
import re

#pattern = r'(\d+)\s+\((\d+),(\d+)\)\s+([\d.]+)'
pattern = r'(\d+)\t\((\d+),(\d+)\)\t(-?\d+\.\d+)' #for SLAM output format

spoke_regex = r"Q_Header<\(([^)]+)\)>\s+Q_Data<\(([^)]+)\)>\s+ORIENT<geometry_msgs\.msg\.Quaternion\(x=([-0-9\.e]+),\s*y=([-0-9\.e]+),\s*z=([-0-9\.e]+),\s*w=([-0-9\.e]+)\)>\s+ANG_VEL<geometry_msgs\.msg\.Vector3\(x=([-0-9\.e]+),\s*y=([-0-9\.e]+),\s*z=([-0-9\.e]+)\)>\s+LIN_ACC<geometry_msgs\.msg\.Vector3\(x=([-0-9\.e]+),\s*y=([-0-9\.e]+),\s*z=([-0-9\.e]+)\)>\s+COMP<(\d+)>"

class CustomOdometryPublisherIMU(Node):
    def __init__(self):
        super().__init__('custom_odometry_publisher_imu')

        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        reentrant_callback_group = ReentrantCallbackGroup() 

        self.timer = self.create_timer(0.2, self.publish_odometry, reentrant_callback_group)

        self.imu_timer = self.create_timer(0, self.publish_imu, reentrant_callback_group)

        self.udp_ip = ''
        self.udp_port = 9998 #As defined for IMU in ControlStation script
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.udp_ip,self.udp_port))
        self.sock.settimeout(0.1)
        self.pose_index = 0
        self.pose_list = []

        self.prev_content = ''

        # Covariance numbers based on BNO055

        self.orientation_cov = [
            0.0159, 0.0,   0.0,
            0.0,   0.0159, 0.0,
            0.0,   0.0,    0.0159
        ]
        self.angular_velocity_cov = [
            0.04, 0.0,  0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0,  0.04
        ]
        self.linear_accel_cov = [ #0.017 default
            0.017, 0.0,   0.0,
            0.0,   0.017, 0.0,
            0.0,   0.0,   0.017
        ]

    def publish_imu(self):

        try:
            data, addr = self.sock.recvfrom(4096)
            #print("GOT IMU DATA")
        except socket.timeout:
            return

        match = re.search(spoke_regex, data.decode().strip())

        if match:
            # Extract matched data using named groups
            id = { #id = imu data
                "Q_Header": match.group(1),
                "Q_Data": match.group(2),
                "ORIENT_x": match.group(3),
                "ORIENT_y": match.group(4),
                "ORIENT_z": match.group(5),
                "ORIENT_w": match.group(6),
                "ANG_VEL_x": match.group(7),
                "ANG_VEL_y": match.group(8),
                "ANG_VEL_z": match.group(9),
                "LIN_ACC_x": match.group(10),
                "LIN_ACC_y": match.group(11),
                "LIN_ACC_z": match.group(12),
                "COMP": match.group(13),
            }
        else:
            return
        
        #print("MATCHED")
        now = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'

        # Simulated orientation as a slowly rotating quaternion about Z axis
        #q = tf_transformations.quaternion_from_euler(0, 0, self.angle)
        imu_msg.orientation = Quaternion(x=float(id["ORIENT_x"]), 
                                         y=float(id["ORIENT_y"]), 
                                         z=float(id["ORIENT_z"]), 
                                         w=float(id["ORIENT_w"]))
        
        # Angular velocity
        imu_msg.angular_velocity.x = float(id["ANG_VEL_x"])
        imu_msg.angular_velocity.y = float(id["ANG_VEL_y"])
        imu_msg.angular_velocity.z = float(id["ANG_VEL_z"])
        
        # Linear acceleration (gravity + small noise)
        imu_msg.linear_acceleration.x = float(id["LIN_ACC_x"])*0.05
        imu_msg.linear_acceleration.y = float(id["LIN_ACC_y"])*0.05
        imu_msg.linear_acceleration.z = float(id["LIN_ACC_z"])*0.05

        imu_msg.orientation_covariance = self.orientation_cov
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov
        imu_msg.linear_acceleration_covariance = self.linear_accel_cov
        
        self.imu_pub.publish(imu_msg)
        

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

        if (content == self.prev_content):
            return
        
        self.prev_content = content
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

        #self.pose_index += 1 #ONLY FOR SIM POSE LIST 

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

    #print("START")

    rclpy.init(args=args)
    node = CustomOdometryPublisherIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
