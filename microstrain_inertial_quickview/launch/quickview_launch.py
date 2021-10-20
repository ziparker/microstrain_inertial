import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
      return LaunchDescription([
            # Declare arguments with default values
            DeclareLaunchArgument('node_name', default_value='gx5'),

           # ****************************************************************** 
           # Quickview Node
           # ****************************************************************** 
           Node(
                 name="microstrain_inertial_quickview",
                 package="rqt_gui",
                 executable="rqt_gui",
                 arguments=[
                       "--force-discover",
                       "--perspective-file",
                       os.path.join(get_package_share_directory('microstrain_inertial_quickview'), 'microstrain_inertial_quickview_common/resource/quickview.perspective'),
                 ]
           )
      ])
  

 
 
