from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Play the rosbag
    ExecuteProcess(
        cmd=[
            'source', '/home/dale/driverless-packages/iSAM2_SLAM/workspace/install/setup.bash'
            ],
        output='screen'
    ),
    ExecuteProcess(
        cmd=[                
            'ros2', 'bag', 'play', '/home/dale/rosbags/controls_sim_bag'
        ],
        output='screen'
    ),
    ExecuteProcess(
        cmd=[
            'python3', '/home/dale/driverless-packages/iSAM2_SLAM/workspace/viz/visualize.py'
        ],
        output='screen'
    )
])
