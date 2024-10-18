from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '/home/jay/hunter_sim_ws/src/hunter_se_gazebo/world/osm_floor.world'
            ],
            output='screen'
        )
    ])
