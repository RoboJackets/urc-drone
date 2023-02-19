from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    aruco_backup_node = Node(
            package='urc_aruco',
            executable='urc_aruco_ArucoBackup',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('urc_aruco'), 'config',
                                     'aruco_backup_params.yaml'])
            ]
        )

    return LaunchDescription([
        aruco_backup_node
    ])
