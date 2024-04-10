from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_name='robot_car_pkg' 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')}]
        )
    ])