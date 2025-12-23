#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Nav2 paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = '/home/singularity/robpractice/maps/my_map.yaml'
    
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
    
    # --- 6. ROS-Gazebo Bridge ---
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': True}],
    )


    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=['/home/singularity/robpractice/config/ekf.yaml'],
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
            parameters=[{'use_sim_time': True}],
        )]
    )
    
    # --- 8. Nav2 Bringup (delayed to let Gazebo stabilize) ---
    nav2_bringup = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to fully start
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'True',
                    'autostart': 'True',  
                    'params_file': '/home/singularity/robpractice/config/nav2_params.yaml',
                }.items()
            )
        ]
    )
    
    # --- 9. RViz2 with Nav2 config ---
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = TimerAction(
        period=6.0,  # Launch after Nav2
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
            )
        ]
    )
    
    return LaunchDescription([
        ign_gazebo,
        rsp_node,
        bridge_node,
        ekf_node,
        spawn_entity_node,
        nav2_bringup,
        rviz_node,
    ])