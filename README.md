# IA368ii
Official repository for the class IA368ii 2025S2:
- `ia368_pkg`: main package with YOLO detection and CoppeliaSim remote API nodes.
- Nodes for Kinect, TF, YOLO 3D detection, dummy creation, and autodocking.

---

## Clone the repository::
```bash
git clone https://github.com/cesarbds/IA368ii.git
```
## Build the Workspace
Use **colcon** to build all packages:
```bash
cd IA368ii
colcon build
source install/setup.bash
```
**You need to source this every new terminal session before running ROS 2 commands.**

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

