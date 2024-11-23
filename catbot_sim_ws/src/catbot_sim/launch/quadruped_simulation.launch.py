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
            cmd=['ros2 run gazebo_ros spawn_entity.py -entity quadruped -file /home/nicoleibert/ROBOTICS_PROJECTS/catbot-simulation/catbot_sim_ws/src/catbot_sim/meshes/catbot.xacro'],
            output='screen'
        ),

        # Quadruped Controller Node
        Node(
            package='catbot_sim',
            executable='quadruped_controller_node',
            name='quadruped_controller',
            output='screen'
        ),

        # # Joystick Controller Node
        # Node(
        #     package='catbot_sim',
        #     executable='joystick_controller_node',
        #     name='joystick_controller',
        #     output='screen'
        # )
        
        # Keyboard Controller Node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_twist',
            output='screen'
        )
    ])
