import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  xacro_file_name = 'robotRuch.urdf.xacro.xml'
  rviz2_file_name = 'robot.rviz'
  xacro = os.path.join(get_package_share_directory('lab5'), xacro_file_name)
  rviz2 = os.path.join(get_package_share_directory('lab5'), rviz2_file_name)
  
  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),

      Node(
          package='lab5',
          executable='ikin',
          name='ikin',
          output='screen',
          ),
  ])

