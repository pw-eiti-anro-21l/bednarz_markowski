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
  xacro = os.path.join(get_package_share_directory('lab3'), xacro_file_name)
  rviz2 = os.path.join(get_package_share_directory('lab3'), rviz2_file_name)
  
  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{
          	'use_sim_time': use_sim_time,
          	'robot_description': Command(['xacro',' ',xacro])
          }]),

      Node(
          package='rviz2',
          executable='rviz2',
          name='robot_rviz2',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=['-d', rviz2]) , 
      Node(
          package='lab3',
          executable='nonkdl_dkin',
          name='nonkdl_dkin',
          output='screen',
          ) ,   
      
          
      Node(
      	  package='joint_state_publisher_gui',
      	  executable='joint_state_publisher_gui',
          name='joint_state_publisher_gui',)
  ])

