import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('kitti_pub')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_default.rviz')  # Updated directory
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='kitti_pub',
            executable='kitti_pub_node',
            name='kitti_pub_node',
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
