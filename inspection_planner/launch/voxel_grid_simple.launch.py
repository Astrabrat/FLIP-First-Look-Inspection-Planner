from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    team = LaunchConfiguration('team')
    robot = LaunchConfiguration('robot')
    frame_id = LaunchConfiguration('frame_id')

    sim_pcl = Node(
        package='inspection_planner',
        executable='pcl_transformer',
        name='pcl_transformer_sim',
        output='screen',
        parameters=[{
            'input_pointcloud': ['/', robot, '/velodyne_points'],
            'output_pointcloud': ['/', robot,'/filtered_pointcloud'],
            'sensor': 'lidar',
            'frame_id': frame_id,
            'filter_min_x': 0.0,
            'filter_max_x': 10.0,
            'filter_min_y': -10.0,
            'filter_max_y': 10.0,
            'filter_min_z': 0.0,
            'filter_max_z': 3.0,
        }],
    )

    real_pcl = Node(
        package='inspection_planner',
        executable='pcl_transformer',
        name='pcl_transformer_real',
        output='screen',
        parameters=[{
            'input_pointcloud': ['/', robot, '/ouster/points'],
            'output_pointcloud': ['/', robot,'/filtered_pointcloud'],
            'sensor': 'lidar',
            'frame_id': frame_id,
            'filter_min_x': -80.0,
            'filter_max_x': 80.0,
            'filter_min_y': -80.0,
            'filter_max_y': 80.0,
            'filter_min_z': 0.0,
            'filter_max_z': 1.0,
        }],
    )

    sim_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", team, "' == 'kSim'"])),
        actions=[
            PushRosNamespace(robot),
            sim_pcl,
        ],
    )

    real_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", team, "' == 'kReal'"])),
        actions=[
            PushRosNamespace(robot),
            real_pcl,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='kSim'),
        DeclareLaunchArgument('robot', default_value='pelican'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        sim_group,
        real_group,
    ])