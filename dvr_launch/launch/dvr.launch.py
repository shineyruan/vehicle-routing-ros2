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

from ament_index_python import get_package_share_directory
import launch.substitutions
from launch.conditions import IfCondition
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

  num_vehicles = LaunchConfiguration('number_vehicles').perform(context)

  common_param_file = os.path.join(get_package_share_directory('dvr_launch'),
                                   'config/common.param.yaml')
  common_param = DeclareLaunchArgument(
      'common_param_file',
      default_value=common_param_file,
      description='Path to the param file for common settings')

  vehicle_param_file = os.path.join(get_package_share_directory('dvr_launch'),
                                    'config/vehicle.param.yaml')
  vehicle_param = DeclareLaunchArgument(
      'vehicle_param_file',
      default_value=vehicle_param_file,
      description='Path to the param file for Vehicle node')

  list_launch_descriptors = [
      LogInfo(msg='Launching %s vehicles' % num_vehicles), common_param,
      vehicle_param
  ]

  for vid in range(int(num_vehicles)):
    vehicle_node = Node(package='dvr_nodes',
                        executable='vehicle',
                        name='vehicle_{}'.format(vid),
                        parameters=[
                            LaunchConfiguration('vehicle_param_file'),
                            LaunchConfiguration('common_param_file'), {
                                'uid': vid
                            }
                        ])
    list_launch_descriptors.append(vehicle_node)

  vehicle_viz_node = Node(package='dvr_nodes',
                          executable='vehicle_visualization',
                          name='vehicle_visualization',
                          parameters=[
                              LaunchConfiguration('vehicle_param_file'),
                              LaunchConfiguration('common_param_file'), {
                                  'num_vehicles': int(num_vehicles)
                              }
                          ])
  list_launch_descriptors.append(vehicle_viz_node)

  return list_launch_descriptors


def generate_launch_description():
  """Generate launch description with a single component."""

  common_param_file = os.path.join(get_package_share_directory('dvr_launch'),
                                   'config/common.param.yaml')
  common_param = DeclareLaunchArgument(
      'common_param_file',
      default_value=common_param_file,
      description='Path to the param file for common settings')

  #################### Launch Vehicle Node ####################
  num_vehicles_param = DeclareLaunchArgument('number_vehicles',
                                             default_value='10')
  #############################################################

  #################### Launch RViz2 ####################
  rviz_cfg_path = os.path.join(get_package_share_directory('dvr_launch'),
                               'rviz/dvr.rviz')
  with_rviz_param = DeclareLaunchArgument(
      'with_rviz',
      default_value='True',
      description='Launch RVIZ2 in addition to other nodes')
  rviz_cfg_path_param = DeclareLaunchArgument(
      'rviz_cfg_path_param',
      default_value=rviz_cfg_path,
      description='Launch RVIZ2 with the specified config file')
  rviz2 = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=[
          '-d',
          launch.substitutions.LaunchConfiguration("rviz_cfg_path_param")
      ],
      condition=IfCondition(
          launch.substitutions.LaunchConfiguration('with_rviz')),
      remappings=[("initialpose", "/localization/initialpose"),
                  ("goal_pose", "/planning/goal_pose")],
  )
  #######################################################

  #################### Publish Map frame ####################
  map_to_odom_tf_publisher = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="static_map_to_odom_tf_publisher",
      output="screen",
      arguments=["0.0", "0.0", "0.0", "0", "0", "0", "map", "odom"],
  )
  ###########################################################

  #################### Launch World Visualizer ####################
  world_visualizer = Node(
      package='dvr_nodes',
      executable='world_visualization',
      name='world_visualization',
      parameters=[LaunchConfiguration('common_param_file')])
  #################################################################

  return LaunchDescription([
      num_vehicles_param,
      OpaqueFunction(function=launch_vehicles), with_rviz_param,
      rviz_cfg_path_param, rviz2, common_param, world_visualizer,
      map_to_odom_tf_publisher
  ])
