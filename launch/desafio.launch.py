import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    des_pkg_share = get_package_share_directory('desafio_oxebots')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_spawn_model_launch_source = os.path.join(
        ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py'
    )

    tb3_model_path = os.path.join(des_pkg_share, 'models', 'turtlebot3_burger', 'model.sdf')
    world_path = os.path.join(des_pkg_share, 'worlds', 'desafio_world.sdf')
    bridge_config_path = os.path.join(des_pkg_share, 'config', 'bridge_config.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(des_pkg_share, 'models'),
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'desafio_world',
            'topic': '/robot_description',
            'entity_name': 'turtlebot3_burger',
            'x': x_pose,
            'y': y_pose,
            'z': '0.2',
        }.items(),
    )

    maze_models = ['model-1', 'model-2', 'model-3']

    maze_param = LaunchConfiguration('maze_model')

    maze_model = PathJoinSubstitution(
        [
            des_pkg_share,
            'models',
            'maze',
            'model.sdf.temp',
        ]
    )

    def modify_sdf(context):
        """Modify the maze model SDF file based on the selected maze version."""
        maze_model_path = maze_model.perform(context)
        with open(maze_model_path, 'r') as file:
            sdf_content = file.read()
        maze_version = maze_param.perform(context)
        # Replace the placeholder with the actual maze version
        sdf_content = sdf_content.replace('$[version]', f'{maze_version}.dae')
        maze_model_modified_path = maze_model_path.replace('.temp', '')
        with open(maze_model_modified_path, 'w') as file:
            file.write(sdf_content)

        return []

    modify_sdf_action = OpaqueFunction(function=modify_sdf)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='model',
                default_value=tb3_model_path,
                description='Absolute path to robot model file',
            ),
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='True',
                description='Flag to enable use_sim_time',
            ),
            DeclareLaunchArgument(
                name='x_pose',
                default_value='0.0',
                description='Initial x position of the TurtleBot3',
            ),
            DeclareLaunchArgument(
                name='y_pose',
                default_value='-11.0',
                description='Initial y position of the TurtleBot3',
            ),
            DeclareLaunchArgument(
                name='maze_model',
                default_value='model-1',
                choices=maze_models,
                description='Choose the maze model to spawn',
            ),
            modify_sdf_action,
            gz_resource_path,
            robot_state_publisher_node,
            gz_server,
            ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
            ros_gz_bridge,
            spawn_entity,
        ]
    )
