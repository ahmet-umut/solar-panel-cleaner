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

### wor.sdf
The world file that includes the model, sensors.

### pac/
The folder of the ROS2 packet.

## Usage
First, inside the worcre.py change the parameters according to your desires. Then on your _workspace_ (the different blocks are different terminals):
```
gz sim wor.sdf &
ros2 launch lau.py
```
```
colcon build
ros2 run pac nod
```

## nod
The ROS2 node. For fast compilation some import files are commented out. It takes keyboard, sensor and odometry inputs from Gazebo and makes some actions according to those. (Publishes 2 types of commands: linear and angular velocity. Tries to not fall down; on edges, turns for a random amount of time to randomly move onside of the panel.
