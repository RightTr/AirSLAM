import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  pkg_share = get_package_share_directory('air_slam')

  config_path_arg = DeclareLaunchArgument(
    'config_path',
    default_value=os.path.join(pkg_share, 'configs/visual_odometry_online/vo_realsense.yaml'),
    description='Path to visual odometry config file'
  )

  camera_config_path_arg = DeclareLaunchArgument(
    'camera_config_path',
    default_value=os.path.join(pkg_share, 'configs/camera/realsense_d435i.yaml'),
    description='Camera config path'
  )

  model_dir_arg = DeclareLaunchArgument(
    'model_dir',
    default_value=os.path.join(pkg_share, 'output'),
    description='Model output dir'
  )

  saving_dir_arg = DeclareLaunchArgument(
    'saving_dir',
    default_value='/home/pi/Result/Air_SLAM',
    description='Saving results dir'
  )

  visualization_arg = DeclareLaunchArgument(
    'visualization',
    default_value='true',
    description='Enable RViz visualization'
  )

  visual_odometry_online_node = Node(
    package='air_slam',
    executable='visual_odometry_online', 
    name='visual_odometry_online',
    output='screen',
    parameters=[{
      'config_path': LaunchConfiguration('config_path'),
      'camera_config_path': LaunchConfiguration('camera_config_path'),
      'model_dir': LaunchConfiguration('model_dir'),
      'saving_dir': LaunchConfiguration('saving_dir')
    }]
  )

  rviz_node = Node(
    condition=LaunchConfigurationEquals('visualization', 'true'),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'vo_ros2.rviz')],
    output='screen',
  )

  return LaunchDescription([
    config_path_arg,
    camera_config_path_arg,
    model_dir_arg,
    saving_dir_arg,
    visualization_arg,
    visual_odometry_online_node,
    rviz_node
  ])

