from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
  
        Node(
            package='tv_sim',
            executable='plant',
            name='plant_node',
            output='screen'
        ),
        
    
        Node(
            package='tv_sim',
            executable='controller',
            name='controller_node',
            output='screen'
        ),
        

        Node(
            package='tv_sim',
            executable='driver',
            name='driving_node',
            output='screen',
            prefix='xterm -e' # Opens a new window so keyboard input works
        )
    ])