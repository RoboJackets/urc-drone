from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    aruco_detector_node = Node(
            package='urc_aruco',
            executable='urc_aruco_ArucoDetector',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('urc_aruco'), 'config',
                                     'aruco_detector_params.yaml'])
            ],
            remappings=[
                ("/aruco_detector/aruco", "/aruco")
            ]
        )

    return LaunchDescription([
        aruco_detector_node
    ])