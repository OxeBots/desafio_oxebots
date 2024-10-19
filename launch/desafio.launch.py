from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    maze_models = ['model-1', 'model-2', 'model-3']

    x_argument_action = DeclareLaunchArgument(
        name='x_pose',
        default_value='0.0',
        description='Initial x position of the TurtleBot3',
    )

    y_argument_action = DeclareLaunchArgument(
        name='y_pose',
        default_value='-11.0',
        description='Initial y position of the TurtleBot3',
    )

    maze_argument_action = DeclareLaunchArgument(
        name='maze_model',
        default_value='model-1',
        choices=maze_models,
        description='Choose the maze model to spawn',
    )

    desafio_oxebots_dir = FindPackageShare('desafio_oxebots')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    maze = LaunchConfiguration('maze_model')

    maze_model = PathJoinSubstitution(
        [
            desafio_oxebots_dir,
            'models',
            'maze',
            'model.sdf.temp',
        ]
    )

    def modify_sdf(context):
        """Modify the maze model SDF file based on the selected maze version."""
        maze_model_path = maze_model.perform(context)

        # Read the template SDF content
        with open(maze_model_path, 'r') as file:
            sdf_content = file.read()

        # Get the selected maze version
        maze_version = maze.perform(context)

        # Replace the placeholder with the actual maze version
        sdf_content = sdf_content.replace('$[version]', f'{maze_version}.dae')

        # Write the modified SDF content to a new file
        maze_model_modified_path = maze_model_path.replace('.temp', '')
        with open(maze_model_modified_path, 'w') as file:
            file.write(sdf_content)

        return []

    modify_sdf_action = OpaqueFunction(function=modify_sdf)

    world = PathJoinSubstitution(
        [
            desafio_oxebots_dir,
            'worlds',
            'desafio_world.world',
        ]
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [desafio_oxebots_dir, 'launch', 'robot_state_publisher.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desafio_oxebots_dir, 'launch', 'spawn_turtlebot3.launch.py'])
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(x_argument_action)
    ld.add_action(y_argument_action)
    ld.add_action(maze_argument_action)
    ld.add_action(modify_sdf_action)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
