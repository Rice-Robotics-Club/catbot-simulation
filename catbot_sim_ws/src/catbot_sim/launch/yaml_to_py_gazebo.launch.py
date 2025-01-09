import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (Command, LaunchConfiguration)
from launch_ros.actions import (Node, SetParameter)
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import yaml

### Designed to convert a yaml file to a launcheable file that loads gazebo with the right SDFs, bridge, etc...
### ros2 launch yaml_to_py_gazebo.launch.py
def generate_launch_description():
    
    # YAML FILE GOES HERE !!!!!
    current_directory = os.path.dirname(os.path.abspath(__file__))
    yml_file_name = current_directory + "/catbot_gazebo.launch.yml"
    
    with open(yml_file_name, 'r') as file:
        yaml_file = yaml.safe_load(file)
    
    print("YAML FILE = " + yml_file_name + " ---- EDIT yml_file_name IN yaml_to_py_gazebo.launch.py ! ! !")

    
    # Take stuff from yaml
    package_description = str(yaml_file['pkg_name'])
    robot_name = str(yaml_file['robot']['name'])
    meshes_dir = str(yaml_file['robot']['meshes_dir'])
    world_name = str(yaml_file['gazebo']['world'])
    worlds_dir = str(yaml_file['gazebo']['worlds_dir'])
    spawn_x = str(yaml_file['gazebo']['spawn_pos_xyz'][0])
    spawn_y = str(yaml_file['gazebo']['spawn_pos_xyz'][1])
    spawn_z = str(yaml_file['gazebo']['spawn_pos_xyz'][2])
    
    print("yml file values: " + package_description + robot_name + meshes_dir + world_name)
    
    ####### DATA INPUT ##########
    urdf_xacro_file = robot_name + ".xacro"
    package_directory = get_package_share_directory(package_description)

    # # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, meshes_dir)
    worlds_models_path = os.path.join(package_directory, worlds_dir) # add local models path
    gazebo_resource_paths = [install_dir_path, robot_meshes_path, worlds_models_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), meshes_dir, urdf_xacro_file)
    
    ####### DATA INPUT END ##########
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Spawn the Robot #
    declare_spawn_model_name = DeclareLaunchArgument("model_name", default_value= (robot_name + "_robot"),
                                                     description="Model Spawn Name")
    declare_spawn_x = DeclareLaunchArgument("x", default_value=spawn_x,
                                            description="Spawn X")
    declare_spawn_y = DeclareLaunchArgument("y", default_value=spawn_y,
                                            description="Spawn Y")
    declare_spawn_z = DeclareLaunchArgument("z", default_value=spawn_z,
                                            description="Spawn Z")
    
    # Setup to launch the simulator and Gazebo world
    world_filepath = install_dir_path + "/" + package_description + "/" + worlds_dir + "/" + world_name + ".sdf"
    if (len(world_name) == 0):
        world_filepath = "empty.sdf"
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 4 '] + [world_filepath])],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name= robot_name + "_robot_spawn",
        arguments=[
            "-name", LaunchConfiguration("model_name"),
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge #
    # GENERAL GAZEBO MAPPING BLUEPRINT:
    # ign_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="ign_bridge",
    #     arguments=[
    #         "/topic1" + "@ros_msg_type" + "[ign_msg_type", # recieve info from gazebo
    #         "/topic2" + "@ros_msg_type" + "]ign_msg_type", # send info to gazebo
    #         "/topic3" + "@ros_msg_type" + "@ign_msg_type", # gazebo & ros both sending / recieving info
    #     ],
    #     remappings=[
    #         ("/topic1", "/namespace/topic1"),
    #         ("/topic2", "/namespace/topic2"),
    #         ("/topic3", "/namespace/topic3"),
    #     ],
    #     output="screen",
    # )  

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_catbot_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'catbot_controller'],
        output='screen'
    )
    
    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_catbot_controller],
                )
            ),
            bridge,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            robot_state_publisher_node,
            declare_spawn_model_name,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,
        ]
    )