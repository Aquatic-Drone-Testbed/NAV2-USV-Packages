import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

from rclpy.callback_groups import ReentrantCallbackGroup
#import time
import math
#import numpy as np
#import cv2
import re

# Regex to match different parts
regex = r"Q_Header<\(([^)]+)\)>\s+Q_Data<\(([^)]+)\)>\s+ORIENT<geometry_msgs\.msg\.Quaternion\(x=([-0-9\.e]+),\s*y=([-0-9\.e]+),\s*z=([-0-9\.e]+),\s*w=([-0-9\.e]+)\)>\s+ANG_VEL<geometry_msgs\.msg\.Vector3\(x=([-0-9\.e]+),\s*y=([-0-9\.e]+),\s*z=([-0-9\.e]+)\)>\s+LIN_ACC<geometry_msgs\.msg\.Vector3\(x=([-0-9\.e]+),\s*y=([-0-9\.e]+),\s*z=([-0-9\.e]+)\)>\s+COMP<(\d+)>"

data_list = []
spokes = []

file_path = "/home/ws/r2l_data/radar_data.txt"  # Replace with the actual file path

def process_line(line):
    match = re.search(regex, line.strip())

    if match:
        # Extract matched data using named groups
        extracted_data = {
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
        
        #print(extracted_data)
        return extracted_data
    else:
        #print("No match found.")
        return -1

class Rad2Las_Simulator(Node):
    def __init__(self):
        super().__init__('nav_module')

        reentrant_callback_group = ReentrantCallbackGroup()
        
### Change the functions and params below. 
### We need to create a robot_description topic
### and a tf_topic?
### and a laserscan topic
### and we need to broadcast in intervals
### frame id should count up, lets just say 1 to inf
### read in data, form laserscan
### remember max laserscan angle is ~6.28 due to 2*pi. maybe use math.pi
        
        self.scan_publisher = self.create_publisher(
            LaserScan,
            "scan",
            10,
            callback_group=reentrant_callback_group
        )

        self.scan_publish_timer = self.create_timer(
            2.5, 
            self.publish_scan, 
            reentrant_callback_group)
        
        self.scan_index = 0

            
    def publish_scan(self):
        # Create a LaserScan message
        msg = LaserScan()

        # Fill in the fields of the LaserScan message
        msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp
        msg.header.frame_id = "base_scan"  # Coordinate frame (e.g., "base_link", "laser_frame")
        msg.angle_min = 0.0  # Start angle (0 degrees)
        msg.angle_max = math.pi * 2  # End angle (360 degrees)
        msg.angle_increment = math.pi / 125  # 250 spokes per cycle
        msg.time_increment = 0.01  # Time increment (set to 0 if not used)
        msg.scan_time = 2.5  # Time increment (set to 0 if not used)
        msg.range_min = 50.0  # Minimum range (meters)
        msg.range_max = 4000.0 # Maximum range (meters)
        
        # Simulated laser scan data
        # Create a list of ranges (here, we simulate a simple scan)
        # In a real case, the data would come from an actual laser sensor

        r_temp = []
        
        for I in range(0,250):
            #print(self.scan_index)
            r_temp.append(float(spokes[self.scan_index]))
            self.scan_index += 1
            if self.scan_index >= len(spokes):
                self.scan_index = 0

        msg.ranges = r_temp

        # Publish the LaserScan message
        self.scan_publisher.publish(msg)
        self.get_logger().info(f'Publishing LaserScan with {len(msg.ranges)} ranges')

def main(args=None):

    try:
        with open(file_path, 'r') as file:
            for line in file:
                processed_line = process_line(line)
                if processed_line != -1:
                    data_list.append(processed_line)
                #print(processed_line)  # Or do something else with the processed line
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

    for entry in data_list:
        r_arr = entry["Q_Data"].split(", ")
        for I in range(0,len(r_arr)):
            if  int(r_arr[I]) > 100:
                spokes.append(I*7)
                break

    print("Lentgh of spokes: ")
    print(len(spokes))
    rclpy.init(args=args)

    rad2las_sim = Rad2Las_Simulator()

    try:
        rclpy.spin(rad2las_sim)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        rad2las_sim.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
