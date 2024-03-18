# Copyright 2024 Szymon
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    gps_topic = LaunchConfiguration('gps_topic').perform(context)
    usb_port = LaunchConfiguration('usb_port').perform(context)
    
    print(gps_topic, usb_port)
    
    ttl_usb_node = Node(
        package='ttl_usb_ros_pkg',
        executable='ttl_usb_node',
        name='ttl_usb_node',
        parameters=
            [
                {'gps_topic': gps_topic},
                {'usb_port': usb_port}
            ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    return [
        ttl_usb_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    
    add_launch_arg('gps_topic', '~/gps/navsat')
    add_launch_arg('usb_port', '/dev/ttyUSB0')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])