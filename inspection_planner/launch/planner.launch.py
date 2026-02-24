from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    team = LaunchConfiguration('team')
    robot = LaunchConfiguration('robot')

    pkg_share = get_package_share_directory('inspection_planner')
    params_sims = os.path.join(pkg_share, 'config', 'rosparams_sims.yaml')
    params_exp = os.path.join(pkg_share, 'config', 'rosparams_exp.yaml')

    # Static TF publisher (kept identical to ROS 1: identity transform)
    static_tf_sim = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'pelican/odom'],
        output='screen',
    )

    static_tf_real = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'spot/odom'],
        output='screen',
    )

    planner_node = Node(
        package='inspection_planner',
        executable='planner_node.py',
        name='planner_node',
        output='screen',
        parameters=[PythonExpression(["'", params_exp, "' if '", team, "' == 'kReal' else '", params_sims, "'"])],
    )

    # Namespace group to match ROS 1 launch behavior
    sim_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", team, "' == 'kSim'"])),
        actions=[
            static_tf_sim,
            PushRosNamespace(robot),
            planner_node,
        ],
    )

    real_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", team, "' == 'kReal'"])),
        actions=[
            static_tf_real,
            PushRosNamespace(robot),
            planner_node,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='kReal'),
        DeclareLaunchArgument('robot', default_value='husky'),
        sim_group,
        real_group,
    ])
