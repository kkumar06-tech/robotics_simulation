#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Paths ---
    pkg_share = get_package_share_directory("my_scout_description")
    xacro_file = os.path.join(pkg_share, 'urdf', 'scout_mini.xacro')

    pkg_sim = get_package_share_directory('scout_gazebo_sim')
    bridge_config = os.path.join(pkg_sim, 'config', 'bridge_config.yaml')


    gazebo_pkg_share = get_package_share_directory("scout_gazebo_sim")
    world_file = os.path.join(gazebo_pkg_share, 'worlds', 'empty.world')

    # --- 2. Process the Xacro ---
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- 3. Set GZ_SIM_RESOURCE_PATH ---
    install_share = os.path.dirname(pkg_share)
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{install_share}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"

    # --- 4. Launch Ignition Gazebo ---
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_file],
        output='screen',
    )

    # --- 5. Robot State Publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
    )

    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    output='screen',
    arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
    parameters=[{'use_sim_time': True}],
    )

    # --- 7. Spawn the robot ---
    spawn_entity_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'scout_mini',
                '-x', '0', '-y', '0', '-z', '0.2',
            ],
        )]
    )

  
    # --- 8. RViz2 ---
    pkg_sim = get_package_share_directory('scout_gazebo_sim')  # Use simulation package
    rviz_config = os.path.join(pkg_sim, 'rviz', 'scout_view.rviz')

    print(f"RViz config path: {rviz_config}")
    print(f"Config exists: {os.path.exists(rviz_config)}")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        ign_gazebo,
        rsp_node,
        bridge_node,
        spawn_entity_node,
        rviz_node
    ])
