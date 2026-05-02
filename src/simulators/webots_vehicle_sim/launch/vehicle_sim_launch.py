import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher



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

    robot_description_path = os.path.expanduser(
        '~/ros2_ws/src/simulators/webots_vehicle_sim/resource/superb.urdf'
    )
    
    driver = TimerAction(
    period=6.0,
    actions=[
            WebotsController(
                robot_name='Skoda_Superb_Mk1',
                parameters=[
                    {'robot_description': robot_description_path}
                ],
                respawn=True
            )
        ]
)

    return LaunchDescription([
        world_generator,
        driver,
    ])