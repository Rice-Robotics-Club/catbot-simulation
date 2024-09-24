# OnShapeToURDF
Note: I named the robot in this case "dingus". Any reference to the name dingus should be replaced with your robot's name.

In most cases where you need to know how to use URDF files, you need to create a ROS version for a robot you have.
Normally, we have a full assembly of our robots in a CAD system.
These robot models are highly convoluted, and generating an accurate URDF version by hand is no easy feat.
We need a way to export automatically to URDF.
We can use Onshape-to-robot to export automatically to URDF!

## Dependencies
Assuming you've already installed ROS2 Humble on your linux / VM computer.
```
sudo apt-get update
# for onshape-to-robot
sudo pip install onshape-to-robot
sudo apt-get install openscad meshlab
# for rviz / vizualisation
sudo apt install ros-humble-xacro
sudo apt-get -y install joint-state-publisher-gui
```
The Onshape-to-robot Python module needs your API access keys to access your Onshape models.
Get the Onshape API keys here: https://dev-portal.onshape.com/keys.
Press the New Key button, then select all the checkboxes.

It will give you this time and ONLY this time the:
- access key
- private key

Save them in a safe place by copy and paste.

## Create a setup script for setting up keys:
```
cd ~/ros2_ws
touch keys.sh
chmod +x keys.sh
```
In keys.sh:
```
// Obtained at https://dev-portal.onshape.com/keys
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=Your_Access_Key
export ONSHAPE_SECRET_KEY=Your_Secret_Key
```
Check that it is sourcing correctly:
```
cd ~/ros2_ws
source keys.sh
echo $ONSHAPE_API
echo $ONSHAPE_ACCESS_KEY
echo $ONSHAPE_SECRET_KEY
```
Output:
```
https://cad.onshape.com
Your_Access_Key
Your_Secret_Key
```
Create a new package for this robot named dingus_description.
Create a folder named dingus to house all the files needed for this export.
In this folder, dingus, export URDF and its 3D meshes.
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake dingus_description --dependencies urdf xacro
cd dingus_description
mkdir dingus
touch dingus/config.json

mkdir launch
mkdir rviz
```
Modify the CMakelists.txt to install the folders dingus, launch, and rviz:
```
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# INSERT THIS INTO CMakeLists.txt
install(
  DIRECTORY
    rviz
    launch
	dingus
  DESTINATION
    share/${PROJECT_NAME}/
)
# END INSERT

ament_package()
```
Now, add the contents of the config.json file.

In config.json (note: you'll have to delete the comments - comments are not allowed in JSON):
```
{
  "documentId": "536a8051abba5659f38bd6ec", # this ID is found in URL of Onshape file. ex: URL = https://cad.onshape.com/documents/33b91de06ddc91b068fcf725/w/3e9cd5a83630cb75d064813a/e/8e6a230fa3aabb1441b0aa17
  "outputFormat": "urdf",                                                                                                      # YOUR documentId ^^^
  "packageName": "dingus_description/dingus", # path from ROS workspace (ie: robot_ws) to folder where generated urdf is located
  "robotName": "dingus_robot",
  "assemblyName": "dingus_v1" # name of assembly in onshape
}
```
Now you are ready to export.
Execute the following commands:
```
cd ~/ros2_ws/src/dingus_description
# Execute the export
onshape-to-robot dingus
```
Note: if you get an error mentioning numpy version being 0xe instead of 0x10, install a newer version of numpy on your machine (1.23.x)
```
pip install numpy==1.23.0 --force
```
Among generated files, you should have a file named robot.urdf.
Insert the following at the top of robot.urdf (Make this line 1, robot_name should be line 2):
```
<?xml version="1.0"?>
```

## ROS launch
Fill in the code for the ROS2 launch files. Copy launch files from quadruped_description pkg. Rename any reference to "quadruped" to dingus

In launch/dingus.launch.py: Assign package_description, robot_desc_path variables to respective pkg / robot names

In launch/start_rviz.launch.py: Assign package_description, rviz_config_dir variables to respective pkg / robot names 

## Running Generated Robot
In shell 1:
```
cd ~/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --packages-select dingus_description
source install/setup.bash
ros2 launch dingus_description dingus.launch.py
```
In shell 2:
```
cd ~/ros2_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dingus_description start_rviz.launch.py
```
In shell 3:
```
cd ~/ros2_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
When rviz2 boots up, adjust the display accordingly:

1. Go to lower left corner of window, click on Add. This will left you create a new display based on display type or topic.
2. Make sure you're on "By display type", then scroll down until you see RobotModel. Select it and OK.
3. Under Global Options, set the fixed frame to whatever part of the model you want to act as an anchor point. Usually either the robot base or chassis.
4. Under RobotModel, set Description Topic to be /robot_description

Final setup should look something like this:
![image](https://github.com/user-attachments/assets/58de415f-0887-4f96-af30-84f67003111c)

In RVIZ, you should see the robot, and be able to move the joints around using joint_state_publisher_gui.

For more info or if something doesn't work, check out the docs: https://onshape-to-robot.readthedocs.io/en/latest/
