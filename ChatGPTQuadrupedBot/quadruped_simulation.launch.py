from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo Ignition
        ExecuteProcess(
            cmd=['ign gazebo -v 4 empty.sdf'],  # Use a blank world or your custom world file
            output='screen'
        ),
        
        # Spawn robot model (URDF)
        ExecuteProcess(
            cmd=['ros2 run gazebo_ros spawn_entity.py -entity quadruped -file /path/to/your/robot.urdf'],
            output='screen'
        ),

        # Quadruped Controller Node
        Node(
            package='quadruped_joystick_control',
            executable='quadruped_controller',
            name='quadruped_controller',
            output='screen'
        ),

        # Joystick Controller Node
        Node(
            package='quadruped_joystick_control',
            executable='joystick_controller',
            name='joystick_controller',
            output='screen'
        )
    ])
