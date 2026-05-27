import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_robot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')
    controller_manager_name = LaunchConfiguration('controller_manager_name')

    urdf_path = os.path.join(pkg_share, 'model', 'wheel_robot.urdf')
    controller_config_path = os.path.join(pkg_share, 'config', 'controller.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'wheel_robot.rviz')

    try:
        with open(urdf_path, 'r') as file:
            urdf_content = file.read()

        urdf_content = urdf_content.replace(
            '__CONTROLLER_CONFIG_PATH__',
            controller_config_path
        )

        # gazebo_ros2_control이 robot_description을 다시 파싱하므로
        # 한 줄 XML로 정리
        urdf_content = ET.tostring(
            ET.fromstring(urdf_content),
            encoding='unicode'
        )

        print("URDF content loaded successfully.")

    except Exception as e:
        print(f"Failed to read URDF file: {e}")
        urdf_content = ""

    robot_description = {
        'robot_description': ParameterValue(
            urdf_content,
            value_type=str
        )
    }

    world_path = os.path.join(pkg_share, 'worlds', 'astar_mapping_test.world')
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': pause,
            'world': world_path,
            'use_sim_time': use_sim_time,
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description
        ]
    )

    gz_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'wheel_robot',
            '-z', '0.15'
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            controller_manager_name,
        ],
        output='screen',
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'wheel_effort_controller',
            '--controller-manager',
            controller_manager_name,
        ],
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }
        ]
    )

    controller_node = Node(
        package='wheel_robot',
        executable='Wheel_robot',
        name='Wheel_robot',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config_path
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='pause',
            default_value='false',
            description='Start Gazebo paused or unpaused'
        ),

        DeclareLaunchArgument(
            name='controller_manager_name',
            default_value='/controller_manager',
            description='controller_manager node name'
        ),

        gazebo_server,
        gazebo_client,

        robot_state_publisher_node,
        gz_spawn_entity,

        joint_state_broadcaster_spawner,
        wheel_controller_spawner,

        joy_node,
        controller_node,

        rviz_node,
    ])