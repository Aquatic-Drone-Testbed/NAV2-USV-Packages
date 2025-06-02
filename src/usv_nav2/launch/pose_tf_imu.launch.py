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
        default_value='usv_nav2_node',
        description='Name of the node'
    )

    # Lifecycle node
    usv_nav2_node = LifecycleNode(
        package='usv_nav2',
        executable='custom_odom_imu',
        name=node_name,
        namespace='',
        output='screen',
        parameters=[]
    )

    # URDF path
    urdf_file_name = 'dummy_descr.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('usv_nav2'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # EKF path
    ekf_config_name = 'ekf.yaml'
    ekf_path = os.path.join(
        get_package_share_directory('usv_nav2'),
        'config',
        ekf_config_name)

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='usv_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )

    #  EKF node from robot_localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_path],
    )


    # Publish static transform
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Publish static transform
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # # RVIZ2 settings
    # rviz2_config = os.path.join(
    #     get_package_share_directory('usv_nav2'),
    #     'config',
    #     'slam.rviz'
    # )

    # # RVIZ2node
    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=[["-d"], [rviz2_config]]
    # )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_node_name_cmd)

    # Launch Robot_Descriptor
    ld.add_action(rsp_node)

    # Launch tf node
    ld.add_action(usv_nav2_node)

    # Launch EKF node
    ld.add_action(ekf_node)

    # Launch map odom static tf node
    ld.add_action(map_odom_tf)

    # Launch imu static tf publisher node
    ld.add_action(imu_tf)

    # Start RVIZ2
    # ld.add_action(rviz2_node)

    return ld
