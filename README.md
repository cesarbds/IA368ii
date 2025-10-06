# IA368ii
Official repository for IA368ii 2025S2 classes:
- `ia368_pkg`: main package with YOLO detection and CoppeliaSim remote API nodes.
- Nodes for Kinect, TF, YOLO 3D detection, dummy creation, and autodocking.

# How to use

## Clone the repository:

Since this repo contains the whole workspace for IA368ii classes, not just the packages, you don't need to / shouldn't clone it inside your ROS 2 workspace.

```bash
git clone https://github.com/cesarbds/IA368ii.git
```
## Build the Workspace
Use **colcon** to build all packages:
```bash
cd IA368ii/IA368_ws
colcon build
source install/setup.bash
```
Add ```--packages-select ia368_pkg``` to build just the ia368_pkg.

**You need to source this workspace for every new terminal session before running ROS 2 commands. You could add the source command to your ```bashrc``` file.**

### Troubleshooting

If you get the following error:
```
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
```
This error was caused by using setuptools >= 71.0.0. You can fix it by downgrading to 70.x:
```
pip install --upgrade setuptools==70.0.0
```
And building the package again.

## Launch YOLO Detection Nodes
### Initiate the scene **tf_scene.ttt**

The YOLO detection launch file supports a dummy argument (0 or 1) to optionally run the dummy node.
```bash
ros2 launch ia368_pkg yolo_detection.launch.py dummy:=0
```
This will start the following nodes in the yolo_detector namespace:

- kinect_node
- tf_node
- yolo_node
- dummy_creation_node (optional, depending on dummy)

## Launch CoppeliaSim Remote API Bridge (Autodocking)

### Initiate the scene **Evaluation scene3.2_students.ttt**

To run the ROS 2 ↔ CoppeliaSim bridge:
```bash
ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py
```
This will start all autodocking nodes:

- battery_node
- bumper_and_velocity_node
- charging_base_node
- docking_node