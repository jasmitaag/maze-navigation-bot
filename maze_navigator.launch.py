import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_navigator',  # Replace with the name of your package
            executable='maze_navigator',  # This is the Python script 
            name='maze_navigator_node',  # Node name, can be customized
            output='screen',
            parameters=[{'use_sim_time': True}],  # Use simulation time for simulators like Gazebo
            remappings=[  
                ('/cmd_vel', '/robot/cmd_vel'),   
                ('/scan', '/robot/scan'),
                ('/imu', '/robot/imu'),
                ('/odom', '/robot/odom')
            ],
        ),
    ])
