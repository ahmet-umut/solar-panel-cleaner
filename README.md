# Solar Panel Cleaner
You can add the packet _pac_ into your ROS 2 workspace (workspace/src/pac), Other files can be put anywhere, but I prefer to keep them in the workspace folder (workspace/file). I will be assuming this for the following explanation.

## ROS-Gazebo versions
ROS Jazzy (ROS2-j) and Gazebo Harmonic (Gazebo-h)

## files
### con.yaml
This file is for the topic configuration of the project. Gazebo topics are converted to ROS2 topics using this configuration file. lau.py uses this file to launch the ROS-Gazebo bridge.

### lau.py
Uses con.yaml to set the bridge between ROS and Gazebo. Launch it by
```ros2 launch lau.py```
.

### worcre.py
Takes the wortem.sdf and converts it to wor.sdf. We could of course edit the wor.sdf file directly -I put the file for the people who want to directly edit the world file.-; however, I recommend this to abstract the details we are interested in about the world and the model. Also, this python file gives errors whenever it sees wrong XML format.

### wortem.sdf
The template file to be edited and saved as wor.sdf.

### wor.sdf
Explained above

### pac/
The folder of the ROS2 packet.

## Usage
First, inside the worcre.py change the parameters according to your desires. Then on your _workspace_ (the different blocks are different terminals):
```
python worcre.py
gz sim wor.sdf &
ros2 launch lau.py
```
```
colcon build
ros2 run pac nod
