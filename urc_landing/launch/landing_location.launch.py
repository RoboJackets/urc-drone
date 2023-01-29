from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    landing_location_node = Node(
            package='urc_landing',
            executable='urc_landing_LandingLocation',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('urc_landing'), 'config',
                                     'landing_location_params.yaml'])
            ],
            remappings=[
                ("/landing_location/landing", "/landing")
            ]
        )

    return LaunchDescription([
        landing_location_node,
    ])
