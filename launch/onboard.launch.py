import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare arguments for vehicle type and number
    veh_arg = DeclareLaunchArgument(
        'veh',
        default_value=EnvironmentVariable('VEHTYPE', default_value='SQ'),
        description='Vehicle type'
    )
    num_arg = DeclareLaunchArgument(
        'num',
        default_value=EnvironmentVariable('VEHNUM', default_value='01'),
        description='Vehicle number'
    )

    # CSV to replay when traj_type:=Csv. Defaults to the bundled trajectory;
    # override with csv_path:=/abs/path.csv. This overrides the YAML's Csv.csv_path.
    default_csv = os.path.join(
        get_package_share_directory('trajectory_generator_ros2'),
        'ref_traj',
        'model_quadloop_to_3_2_1_back_spline10_autostretch_zplus1.csv'
    )
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value=default_csv,
        description='Absolute path to the Goal CSV to replay (used when traj_type is Csv).'
    )

    # Namespaec from veh and num arguements
    namespace = [LaunchConfiguration('veh'), LaunchConfiguration('num')]

    # Define node
    trajectory_generator_node = Node(
        package='trajectory_generator_ros2',
        namespace=namespace,
        executable='trajectory_generator_ros2',
        name='trajectory_generator_ros2',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('trajectory_generator_ros2'),
                'config',
                'default.yaml'
            ),
            # Override Csv.csv_path so the bundled (or user-supplied) CSV is used
            # without hard-coding an absolute path in the YAML.
            {'Csv.csv_path': LaunchConfiguration('csv_path')},
        ]
    )

    launch_group = GroupAction([trajectory_generator_node])

    return LaunchDescription([
        veh_arg,
        num_arg,
        csv_path_arg,
        launch_group
    ])