from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    world_generator = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='webots_vehicle_sim',
                executable='world_generator',
                name='world_generator',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        world_generator,
    ])