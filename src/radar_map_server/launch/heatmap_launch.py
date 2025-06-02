from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_map_server',
            executable='heatmap_publisher',
            name='heatmap_publisher',
            output='screen',
            parameters=[
                # self explanatory
                #{'heatmap_file':    '/home/ws/map/demo/heatmap_counter_367.npy'},
                {'heatmap_file':    '/home/ws/map/curr_stitched_map_probability_map.npy'},

                # The ROS frame ID under which the OccupancyGrid is published
                #   (i.e. msg.header.frame_id). Usually "map" so that Nav2 and RViz know
                #   it’s the world‐fixed map.
                {'map_frame':       'map'},

                # Size of each grid cell in meters.
                {'resolution':      0.05},

                # The world–coordinate position of your grid’s (0,0) cell, in meters.
                #   i.e. If you want your map’s lower‐left corner at (–10 m, –5 m) in
                #   the “map” frame, you’d set origin_x: -10.0, origin_y: -5.0.
                {'origin_x':        -12.5},
                {'origin_y':        -12.5},

                # (Optional) rotation of the entire grid about Z, in radians.
                # A nonzero yaw would tilt your grid relative to the “map” axes.
                # In our case (2d map) we can set this to 0.0 and publish an identity quaternion instead.
                {'origin_yaw':      0.0},

                # Confidence threshold below which a cell is considered “free” (occupancy = 0).
                #   Any heatmap value ≤ free_thresh indicates a free cell.
                {'free_thresh':     0.1},

                # Confidence threshold above which a cell is considered “occupied” (occupancy = 100).
                #   Any heatmap value ≥ occupied_thresh indicates an occupied cell.
                {'occupied_thresh': 0.5},
            ],
        ),
    ])
