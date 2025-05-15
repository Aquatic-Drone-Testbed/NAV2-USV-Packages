#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid

class HeatmapPublisher(Node):
    def __init__(self):
        super().__init__('heatmap_publisher')

        # --- declare parameters with defaults ---
        self.declare_parameter('heatmap_file',  '')
        self.declare_parameter('map_frame',     'map')
        self.declare_parameter('resolution',    0.05)
        self.declare_parameter('origin_x',      0.0)
        self.declare_parameter('origin_y',      0.0)
        self.declare_parameter('origin_yaw',    0.0)
        self.declare_parameter('free_thresh',   10.0)
        self.declare_parameter('occupied_thresh', 90.0)

        # --- get parameters ---
        fn       = self.get_parameter('heatmap_file').get_parameter_value().string_value
        frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        res      = self.get_parameter('resolution').get_parameter_value().double_value
        ox       = self.get_parameter('origin_x').get_parameter_value().double_value
        oy       = self.get_parameter('origin_y').get_parameter_value().double_value
        oyaw     = self.get_parameter('origin_yaw').get_parameter_value().double_value
        ft       = self.get_parameter('free_thresh').get_parameter_value().double_value
        ot       = self.get_parameter('occupied_thresh').get_parameter_value().double_value

        # --- load and convert heatmap ---
        hm = np.load(fn)             # shape (H, W)
        hm = np.flipud(hm)
        H, W = hm.shape
        data = []
        for y in range(H):
            for x in range(W):
                v = hm[y, x]
                if v >= ot:
                    data.append(100)
                elif v <= ft:
                    data.append(0)
                else:
                    data.append(-1)

        # --- build OccupancyGrid ---
        msg = OccupancyGrid()

        # header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # map metadata
        msg.info.resolution = float(res)
        msg.info.width = W
        msg.info.height = H
        msg.info.origin.position.x = float(ox)
        msg.info.origin.position.y = float(oy)

        # 2d map means no rotation for Z yaw → identity quaternion
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # occupancy data
        msg.data = data

        # --- publisher with transient_local (latched) QoS ---
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)

        # publish once per second
        self.create_timer(1.0, lambda: self._publish(msg))

    def _publish(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().debug('Published /map')

def main(args=None):
    rclpy.init(args=args)
    node = HeatmapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
