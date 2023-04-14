# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    share_dir = get_package_share_directory("cyberdog_occmap")
    config_file = os.path.join(share_dir, "param", "gridmap_node.yaml")
    namespace = LaunchConfiguration("namespace", default="")
    create_gridmap_node = Node(
        package="cyberdog_occmap",
        namespace=namespace,
        executable="create_gridmap_node",
        name="create_gridmap_node",
        output="log",
        emulate_tty=True,
        parameters=[{"config_file": config_file}],
    )
    publish_gridmap_node = Node(
        package="cyberdog_occmap",
        namespace=namespace,
        executable="publish_gridmap_node",
        name="publish_gridmap_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"config_file": config_file}],
    )
    return LaunchDescription(
        [
            create_gridmap_node,
            publish_gridmap_node,
        ]
    )
