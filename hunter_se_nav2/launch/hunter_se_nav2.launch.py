import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #=============================1. Gazebo and Robot Setup========================================
    robot_name_in_model = 'hunter_se'
    package_name = 'hunter_se_gazebo'
    xacro_name = "hunter_se_gazebo_jiajie.xacro"
    world_file_path = 'src/hunter_se_gazebo/world/world'  # 世界文件路径

    # Locate package and xacro model
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    xacro_model_path = os.path.join(pkg_share, f'xacro/{xacro_name}')
    
    urdf_model_path = '/tmp/hunter_se_gazebo.urdf'  # 临时文件存储URDF

    # xacro to urdf conversion
    xacro_process_cmd = ExecuteProcess(
        cmd=['xacro', xacro_model_path, '-o', urdf_model_path],
        output='screen'
    )

    # Launch Gazebo with world file
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    # Spawn robot in Gazebo after 20s delay
    delay_before_spawn = TimerAction(
        period=20.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_name_in_model, '-file', urdf_model_path],
            output='screen'
        )]
    )

    #=============================2. Nav2 and RViz Setup=============================================
    # Package directories
    hunter_se_nav2_dir = get_package_share_directory('hunter_se_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(hunter_se_nav2_dir, 'maps', 'newmap.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(hunter_se_nav2_dir, 'param', 'hunter_se_nav2.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # Include Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items()
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    #=============================3. Combine Actions into Launch Description========================
    ld = LaunchDescription()

    # Add Gazebo and robot spawning actions
    ld.add_action(xacro_process_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(delay_before_spawn)
    ld.add_action(start_robot_state_publisher_cmd)

    # Add Nav2 and RViz actions
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)

    return ld
