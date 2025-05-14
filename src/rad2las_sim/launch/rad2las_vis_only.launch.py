# Copyright 2022 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    
    node_name = LaunchConfiguration('node_name')

    # Launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='rad2las_node',
        description='Name of the node'
    )

    # LDLidar lifecycle node
    rad2las_node = LifecycleNode(
        package='rad2las_sim',
        executable='rad2las_sim',
        name=node_name,
        namespace='',
        output='screen',
        parameters=[]
    )

    # URDF path
    urdf_file_name = 'dummy_descr.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('rad2las_sim'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rad2las_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )

    # Publish fake odometry
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # # SLAM Toolbox configuration for rad2las
    # slam_config_path = os.path.join(
    #     get_package_share_directory('rad2las_sim'),
    #     'params',
    #     'slam_toolbox.yaml'
    # )

    # # SLAM Toolbox node in async mode
    # slam_toolbox_node = LifecycleNode(          
    #       package='slam_toolbox',
    #       executable='async_slam_toolbox_node',
    #       namespace='',
    #       name='slam_toolbox',
    #       output='screen',
    #       parameters=[
    #         # YAML files
    #         slam_config_path # Parameters
    #       ],
    #     #   remappings=[
    #     #       ('/scan', '/custom/scan')
    #     #   ]          
    # )

    # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('rad2las_sim'),
        'config',
        'rad2las_slam.rviz'
    )

    # RVIZ2node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_node_name_cmd)

    # Launch Robot_Descriptor
    ld.add_action(rsp_node)

    # # Launch SLAM Toolbox node
    # ld.add_action(slam_toolbox_node) 

    # Launch rad2las node
    ld.add_action(rad2las_node)

    # Launch fake odom publisher node
    ld.add_action(fake_odom)

    # Start RVIZ2
    ld.add_action(rviz2_node)

    return ld
