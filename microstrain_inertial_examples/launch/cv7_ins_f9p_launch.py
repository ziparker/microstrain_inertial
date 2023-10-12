import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_CV7_INS_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7_ins_f9p', 'cv7_ins.yml')
_F9P_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7_ins_f9p', 'f9p.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7_ins_f9p', 'display.rviz')

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _CV7_INS_PARAMS_FILE,
        'namespace': '/',
        'debug': 'true',
      }.items()
    ),

    # Remap the ublox topics to the ones needed by the microstrain driver
    SetRemap('/fix', '/ext/llh_position'),
    SetRemap('/fix_velocity', '/ext/velocity_enu'),

    # Ublox node
    Node(
      package='ublox_gps',
      executable='ublox_gps_node',
      output='screen',
      parameters=[
        _F9P_PARAMS_FILE
      ]
    ),

    # Publish a static transform for where the F9P is mounted on base_link.
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "0.263",
          "--y", "0.01",
          "--z", "1.0",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "f9p_link"
        ]
    ),

    # Run rviz to view the state of the application
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=[
        '-d', _RVIZ_DISPLAY_FILE
      ]
    ),
  ])