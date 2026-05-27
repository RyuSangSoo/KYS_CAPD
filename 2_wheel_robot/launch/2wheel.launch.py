import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xml.etree.ElementTree as ET
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='wheel_robot').find('wheel_robot')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    controller_manager_name = LaunchConfiguration('controller_manager_name')

    urdf_path = os.path.join(pkg_share, 'model/wheel_robot.urdf')
    controller_config_path = os.path.join(pkg_share, 'config', 'controller.yaml')

    try:
        with open(urdf_path, 'r') as file:
            urdf_content = file.read()
        urdf_content = urdf_content.replace('__CONTROLLER_CONFIG_PATH__', controller_config_path)
        # gazebo_ros2_control re-parses robot_description as a parameter override.
        # A single-line XML string avoids parse failures with raw multi-line URDF text.
        urdf_content = ET.tostring(ET.fromstring(urdf_content), encoding='unicode')
        print("URDF content loaded successfully.")
    except Exception as e:
        print(f"Failed to read URDF file: {e}")
        urdf_content = ""

    robot_description = {
        'robot_description': ParameterValue(urdf_content, value_type=str)
    }
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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

    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description]
    )

    gz_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'wheel_robot',
                   '-z', '0.2'],
                #    '-z', '0.8'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    controller_node = launch_ros.actions.Node(
        package='wheel_robot',
        executable='Wheel_robot',
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(
            name='controller_manager_name',
            default_value='/controller_manager',
            description='controller_manager node name',
        ),
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        joy_node,
        controller_node

    ])
