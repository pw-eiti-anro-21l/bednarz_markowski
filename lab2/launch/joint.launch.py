import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  xacro_file_name = 'robotRuch.urdf.xacro.xml'
  rviz2_file_name = 'r2d2.rviz'
  xacro = os.path.join(get_package_share_directory('lab2'), xacro_file_name)
  rviz2 = os.path.join(get_package_share_directory('lab2'), rviz2_file_name)
  
  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),

      Node(
          package='lab2',
          executable='joint_state_publisher',
          name='joint_state_publisher',
          output='screen'
          ),
   
  ])

