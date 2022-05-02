# Copyright 2021 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
"""Launch file for vehicle for Autoware bootcamp (Lincoln MKZ @Pennovation)."""

import os

from pytest import param
from ament_index_python import get_package_share_directory
import launch.substitutions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def get_shared_file_path(package_name, folder_name, file_name):
  """Get the full path of the shared file."""
  return os.path.join(get_package_share_directory(package_name), folder_name,
                      file_name)


def get_shared_file(package_name, folder_name, file_name, arg_name):
  """Pass the given shared file as a LaunchConfiguration."""
  file_path = os.path.join(get_package_share_directory(package_name),
                           folder_name, file_name)
  return launch.substitutions.LaunchConfiguration(arg_name,
                                                  default=[file_path])


def launch_vehicles(context, *args, **kwargs):
  """
  Launch the expected number of vehicle nodes with unique ID.

  Trick borrowed from https://answers.ros.org/question/382000/ros2-makes-launch-files-crazy-too-soon-to-be-migrating/?answer=386056#post-id-386056
  """

  num_vehicles = LaunchConfiguration('number_of_vehicle').perform(context)

  list_launch_descriptors = [
      LogInfo(msg='Launching %s vehicles' % num_vehicles)
  ]

  for vid in range(int(num_vehicles)):
    vehicle_node = Node(package='dvr_nodes',
                        executable='vehicle',
                        name='vehicle_{}'.format(vid),
                        parameters=[{
                            'uid': vid
                        }])
    list_launch_descriptors.append(vehicle_node)

  return list_launch_descriptors


def generate_launch_description():
  """Generate launch description with a single component."""

  num_vehicles_param = DeclareLaunchArgument('number_of_vehicle',
                                             default_value='10')

  return LaunchDescription(
      [num_vehicles_param,
       OpaqueFunction(function=launch_vehicles)])
