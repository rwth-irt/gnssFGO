import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def get_params(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    logger = LaunchConfiguration("log_level")
    share_dir = get_package_share_directory('online_fgo')
    xacro_path = os.path.join(share_dir, 'config/aachen_tc', 'dienstwagen.urdf.xacro')

    config_common_path = LaunchConfiguration('config_common_path')
    default_config_common = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/aachen_tc',
        'common.yaml'
    )

    default_config_integrator = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/aachen_tc',
        'integrator.yaml'
    )

    default_config_optimizer = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/aachen_tc',
        'optimizer.yaml'
    )

    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_common,
        description='CommonParameters')

    declare_config_integrtor_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_integrator,
        description='IntegratorParameters')

    declare_config_optimizer_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_optimizer,
        description='OptimizerParameters')

    online_fgo_node = Node(
        package='online_fgo',
        executable='online_fgo_node',
        name="online_fgo",
        namespace="deutschland",
        output='screen',
        emulate_tty=True,
        # prefix=['gdb -ex run --args'],
        # arguments=['--ros-args', '--log-level', logger],
        parameters=[
            config_common_path,
            default_config_common,
            default_config_integrator,
            default_config_optimizer,
            {

            }
            # Overriding
            # {
            # }
        ],
        remappings=[
            ('/irt_gpio/novatel/pps', '/irt_gpio_novatel/jetson_pps')
        ]
    )

    robot_des = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path])
        }]
    )

    plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name="rqt_plot_fgo",
        output='screen',
        # arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {
                "use_sim_time": True
            }
            # Overriding
            # {
            # }
        ]  # ,
    )
    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value=["debug"],
        description="Logging level"))
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_config_integrtor_path_cmd)
    ld.add_action(declare_config_optimizer_path_cmd)
    ld.add_action(online_fgo_node)
    ld.add_action(robot_des)
    # ld.add_action(plot_node)

    return ld
