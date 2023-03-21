import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nav2_pkg'),
        'launch',
        'nav2_params.yaml'
        )
        
    hokuyo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('urg_node2'), 'launch'),
         '/urg_node2_2lidar.launch.py'])
      )
    
    urdf_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_description'), 'launch'),
         '/simbot_urdf.launch.py'])
      )
      
    nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav2_pkg'), 'launch'),
         '/nav2_launch.py'])
      )
      
    return LaunchDescription([
        # ARIA     
        Node(
            package = 'ros2aria',
            name = 'ros2aria',
            executable = 'ros2aria',
            output = 'screen'),
        # HOKUYO LIDAR
        hokuyo_launch,
        
        # URDF
        urdf_launch,
        
        # NAVIGATION
        nav2_launch  
        
    ])
