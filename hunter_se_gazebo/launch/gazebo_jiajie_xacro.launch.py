import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    robot_name_in_model = 'hunter_se'
    package_name = 'hunter_se_gazebo'
    xacro_name = "hunter_se_gazebo_jiajie.xacro"  # 修改为xacro文件
    world_file_path = 'src/hunter_se_gazebo/world/osm_floor.world'  # 世界文件路径

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    xacro_model_path = os.path.join(pkg_share, f'xacro/{xacro_name}')

    # 使用xacro解析器将xacro转换为urdf
    urdf_model_path = '/tmp/hunter_se_gazebo.urdf'  # 临时文件存储URDF

    xacro_process_cmd = ExecuteProcess(
        cmd=['xacro', xacro_model_path, '-o', urdf_model_path],
        output='screen'
    )

    # 启动Gazebo服务器并加载特定的世界文件
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 启动Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    delay_before_spawn = TimerAction(
        period=30.0,  # 延迟30秒
        actions=[
            # 在Gazebo中生成实体
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot_name_in_model,
                    '-file', urdf_model_path,
                    '-x', '172.0',
                    '-y', '53.0',
                    '-z', '0.5',
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # 启动RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # 在Gazebo中生成实体
# 在Gazebo中生成实体，并指定初始位置和朝向
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', urdf_model_path,
            '-x', '172.0',  # X 坐标
            '-y', '53.0',  # Y 坐标
            '-z', '0.5',  # Z 坐标
            '-R', '0.0',  # Roll
            '-P', '0.0',  # Pitch
            '-Y', '0.0'  # Yaw (弧度，例如 1.57 约等于 90 度)
        ],
        output='screen'
    )


    # 添加所有动作到LaunchDescription中
    ld.add_action(xacro_process_cmd)  # 先运行xacro解析
    ld.add_action(start_gazebo_cmd)
    ld.add_action(delay_before_spawn)  # 使用延迟来等待Gazebo启动
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
