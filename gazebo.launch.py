import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='tortoisebot_gazebo').find('tortoisebot_gazebo')
    world_path = os.path.join(pkg_share, 'worlds/room2.sdf')

    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Set default value to 'true'
    gui = LaunchConfiguration('gui')
    # Set default value to 'false'
    headless = LaunchConfiguration('headless')

    return launch.LaunchDescription([

        # Declare launch arguments
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false',
            description='Flag to enable Gazebo GUI'
        ),
        launch.actions.DeclareLaunchArgument(
            name='headless',
            default_value='true',
            description='Flag to enable headless mode'
        ),

        # Execute Gazebo with specified arguments
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen',
            additional_env={'GAZEBO_PLUGIN_PATH': os.environ['GAZEBO_PLUGIN_PATH'],
                            'GAZEBO_MODEL_PATH': os.environ['GAZEBO_MODEL_PATH'],
                            'GAZEBO_MASTER_URI': os.environ['GAZEBO_MASTER_URI']},
            # Pass the gui and headless arguments to Gazebo
            arguments=['--', '-gui' if gui == 'true' else '',
                       '--headless' if headless == 'true' else '']
        ),


        # Spawn Gazebo entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'tortoisebot',
                       '-topic', 'robot_description'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
