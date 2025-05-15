import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy.callback_groups import ReentrantCallbackGroup
import socket

#import time
import math
#import numpy as np
#import cv2

class Cmd_Vel2ThrustModule(Node):
    def __init__(self):
        super().__init__('cmd_vel2thrust')

        self.udp_ip = ''
        self.udp_port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #reentrant_callback_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.get_cmd_vel,  #receives messages/ processes controller input``
            10)
        
        
        self.subscription  # Prevent unused variable warning


    def get_cmd_vel(self, msg):
        linear = msg.linear
        angular = msg.angular
        self.get_logger().debug(
            f'Linear: x={linear.x:.2f}, y={linear.y:.2f}, z={linear.z:.2f} | '
            f'Angular: x={angular.x:.2f}, y={angular.y:.2f}, z={angular.z:.2f}'
        )

        message = f"{msg.linear.x},{msg.linear.y},{msg.linear.z}," \
                  f"{msg.angular.x},{msg.angular.y},{msg.angular.z}"
        
        self.sock.sendto(message.encode('utf-8'), (self.udp_ip, self.udp_port))

        self.get_logger().debug(f'Sent UDP: {message}')

def main(args=None):
    rclpy.init(args=args)

    cmd_vel2thrust = Cmd_Vel2ThrustModule()

    try:
        rclpy.spin(cmd_vel2thrust)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        cmd_vel2thrust.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
