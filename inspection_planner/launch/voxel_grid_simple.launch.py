from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    team = LaunchConfiguration('team')
    robot = LaunchConfiguration('robot')
    frame_id = LaunchConfiguration('frame_id')

    # Sim params (copied from ROS 1 launch)
    sim_pcl = Node(
        package='inspection_planner',
        executable='pcl_transformer',
        name='pcl_transformer',
        output='screen',
        parameters=[{
            'input_pointcloud': '/pelican/velodyne_points',
            'output_pointcloud': 'filtered_pointcloud',
            'sensor': 'lidar',
            'frame_id': frame_id,
            'filter_min_x': 0.0,
            'filter_max_x': 10.0,
            'filter_min_y': -10.0,
            'filter_max_y': 10.0,
            'filter_min_z': -8.0,
            'filter_max_z': 8.0,
        }],
    )

    # Real params (copied from ROS 1 launch)
    real_pcl = Node(
        package='inspection_planner',
        executable='pcl_transformer',
        name='pcl_transformer',
        output='screen',
        parameters=[{
            'input_pointcloud': '/spot/ouster/points',
            'output_pointcloud': '/spot/filtered_pointcloud',
            'sensor': 'lidar',
            'frame_id': frame_id,
            # Note: original ROS 1 file had max/min swapped for x; kept as-is.
            'filter_min_x': 10.0,
            'filter_max_x': -10.0,
            'filter_min_y': -10.0,
            'filter_max_y': 0.0,
            'filter_min_z': 0.0,
            'filter_max_z': 8.0,
        }],
    )

    sim_group = GroupAction(
        condition=IfCondition(PythonExpression([team, " == 'kSim'"])),
        actions=[
            PushRosNamespace(robot),
            sim_pcl,
        ],
    )

    real_group = GroupAction(
        condition=IfCondition(PythonExpression([team, " == 'kReal'"])),
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
