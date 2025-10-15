# IA368ii
Official repository for IA368ii 2025S2 classes. This repository contains a ROS 2 workspace named `IA368_ws`, which constains packages developed during the classes. The workspace contains:
- `ia368_pkg`: main package with YOLO detection, CoppeliaSim remote API nodes and position control nodes, including their respective scenes (for now).
    - Nodes for Kinect, TF, YOLO 3D detection, dummy creation, autodocking and position control nodes.

# Table of Content

1. [How to use the workspace](#how-to-use-the-workspace)
    1. [Installing dependencies](#installing-dependencies)
    2. [Clone the repository](#clone-the-repository)
    3. [Build the workspace](#build-the-workspace)
        1. [Troubleshooting](#troubleshooting)
2. [Launch YOLO Detection Nodes](#launch-yolo-detection-nodes)
    1. [Installing dependencies (Ubuntu 22.04)](#installing-dependencies-ubuntu-2204)
    2. [Installing dependencies (Ubuntu 24.04)](#installing-dependencies-ubuntu-2404)
        1. [Troubleshooting](#troubleshooting-1)
    3. [Initiate the scene **tf_scene.ttt**](#initiate-the-scene-tf_scenettt)
3. [Launch CoppeliaSim Remote API Bridge (Autodocking)](#launch-coppeliasim-remote-api-bridge-autodocking)
    1. [Initiate the scene **Evaluation scene3.2_students.ttt**](#initiate-the-scene-evaluation-scene32_studentsttt)
4. [Launch Position Control Node](#launch-position-control-node)
    1. [Initiate the scene ***Exercise_position_control.ttt***](#initiate-the-scene-exercise_position_controlttt)
        1. [Troubleshooting](#troubleshooting-2)

# How to use the workspace

## Installing dependencies

A common dependency for all scripts in this repository is the `coppeliasim-zmqremoteapi-client` library. Use **pip** to install it:
```
pip install coppeliasim-zmqremoteapi-client
```
If you are using Ubuntu >= 24.04, you may need to use the `--break-system-packages`option:
```
pip install coppeliasim-zmqremoteapi-client --break-system-packages
```

## Clone the repository

Since this repo contains the whole workspace for IA368ii classes, not just the packages, you don't need to / shouldn't clone it inside your ROS 2 workspace.

```bash
git clone https://github.com/cesarbds/IA368ii.git
```
## Build the workspace
Use **colcon** to build all packages:
```bash
cd IA368ii/IA368_ws
colcon build
source install/setup.bash
```
Add ```--packages-select ia368_pkg``` to build just the package `ia368_pkg`.

**You need to source this workspace for every new terminal session before running ROS 2 commands. You could add the source command to your ```bashrc``` file.**

### Troubleshooting

If you get the following error:
```
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
```
This error may be caused by using `setuptools >= 71.0.0`. You can fix it by downgrading to `70.x`:
```
pip install --upgrade setuptools==70.0.0
```
And building the package again.

# Launch YOLO Detection Nodes
## Installing dependencies (Ubuntu 22.04)

Use **pip** to install **ultralytics** (this may take a few minutes):
```
pip install ultralytics
```
## Installing dependencies (Ubuntu 24.04)

Again, use **pip** to install **ultralytics**, but with some differences (kindly informed by Alfredo Dal'Ava Júnior):
```
pip install ultralytics --no-deps --break-system-packages
pip install polars requests torchvision ultralytics-thop opencv-python numpy==1.26.4 --break-system-packages
pip install torch --break-system-packages
```
### Troubleshooting
If you get the following error:
```
The package's contents are unknown: no RECORD file was found for Numpy/SciPy.
```
You could use some tips from [Stack Overflow](https://stackoverflow.com/questions/68886239/cannot-uninstall-numpy-1-21-2-record-file-not-found). Basically, you need to manually delete the Numpy/SciPy files/directories and reinstall it.

## Initiate the scene **tf_scene.ttt**

The CoppeliaSim scene is available [here](IA368_ws/src/ia368_pkg/yolo_detector/tf_scene.ttt). The YOLO detection launch file supports a dummy argument (0 or 1) to optionally run the dummy node.
```bash
ros2 launch ia368_pkg yolo_detection.launch.py dummy:=0
```
This will start the following nodes in the yolo_detector namespace:

- kinect_node
- tf_node
- yolo_node
- dummy_creation_node (optional, depending on dummy)

# Launch CoppeliaSim Remote API Bridge (Autodocking)

## Initiate the scene **Evaluation scene3.2_students.ttt**

The CoppeliaSim scene is available [here](<IA368_ws/src/ia368_pkg/autodocking/Evaluation scene3.2_students.ttt>). You need to move this scene to your `roomba docking` directory. To run the ROS 2 ↔ CoppeliaSim bridge:
```bash
ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py
```
This will start all autodocking nodes:

- battery_node
- bumper_and_velocity_node
- charging_base_node
- docking_node

# Launch Position Control Node

## Initiate the scene **Exercise_position_control.ttt**

The CoppeliaSim scene is available [here](IA368_ws/src/ia368_pkg/position_control/Exercise_position_control.ttt). To run ROS 2 position control scripts:
```
ros2 launch ia368_pkg position_control.launch.py
```
This will start all position control nodes:

- odom_node
- position_control_node_students (you should complete this one with your code)
- target_node
- velocity_node

### Troubleshooting
If myRobot starts moving without ROS 2 control, change the *Target velocity*, in *Dynamic properties*, of each motor (rightMotor and leftMotor).