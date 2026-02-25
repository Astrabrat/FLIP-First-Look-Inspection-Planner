from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    team = LaunchConfiguration('team')   # 'kSim' or 'kReal'
    robot = LaunchConfiguration('robot') # e.g. 'husky', 'pelican', 'spot'

    pkg_share = get_package_share_directory('inspection_planner')
    params_sims = os.path.join(pkg_share, 'config', 'rosparams_sims.yaml')
    params_exp  = os.path.join(pkg_share, 'config', 'rosparams_shafter.yaml')

    # --- Static TF publishers ---
    # NOTE: If you push namespace, these frame names should usually be relative (no leading '/')
    # and include the robot prefix if you want them namespaced.
    static_tf_sim = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        # world -> <robot>/odom (robot resolved by GroupAction namespace push)
        # With PushRosNamespace(robot), you typically want frames WITHOUT the robot prefix:
        # e.g. 'odom' not 'pelican/odom'
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
    )

    static_tf_real = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
    )

    # --- Planner node ---
    # IMPORTANT: We do NOT set namespace=robot here because we already PushRosNamespace(robot).
    # Doing both would double-namespace: /husky/husky/inspection_planner_node
    planner_node = Node(
        package='inspection_planner',
        executable='planner_node.py',
        name='inspection_planner_node',
        output='screen',
        # Pick param file based on team (works correctly; outputs a *string path*)
        parameters=[params_exp],
    )

    # --- Namespaced groups ---
    sim_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", team, "' == 'kSim'"])),
        actions=[
            PushRosNamespace(robot),
            static_tf_sim,
            planner_node,
        ],
    )

    real_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", team, "' == 'kReal'"])),
        actions=[
            PushRosNamespace(robot),
            static_tf_real,
            planner_node,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='kReal', description="kReal or kSim"),
        DeclareLaunchArgument('robot', default_value='shafter', description="robot namespace"),
        sim_group,
        real_group,
    ])