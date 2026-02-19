# ROS2 Wrapper for Anki Vector

[![Watch my YouTube video](https://img.youtube.com/vi/upcY2gf1lsA/0.jpg)](https://www.youtube.com/watch?v=upcY2gf1lsA&t=3s)

---

## Topics

Current ROS 2 topics being published:

- `/odom`  
- `/imu/data`  
- `/camera/image_raw`  
- `/joint_states` (work in progress)

---

## Setup

### 1. Prepare Virtual Machine

- Use **VirtualBox** and create a VM with **Ubuntu 22.04 Desktop** ISO.  
- Set memory to **8012 MB** and CPUs to **4**.  
- Install **ROS 2 Humble** following the official ROS 2 documentation.

### 2. Workspace Setup

```bash
# Make your workspace directory
mkdir -p ~/ROS2_PROJECT/vector_ws
cd ~/ROS2_PROJECT/vector_ws

# Clone this repository
git clone <your-repo-url>

# Install Python dependencies
pip3 install -r requirements.txt
```
## Configure Vector 

```bash
python3 -m anki_vector.configure
cd src/vector_driver/vector_driver
code .
# Edit vector_node.py on line 76 and replace with your robot's serial number
```
## Build the Workspace 
```bash
# Source ROS2 environment 
source /opt/ros/humble/setup.bash

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```
## Running 
### Launch Nodes 
```bash
# Terminal 1: publish sensor readings 
ros2 run vector_driver vector_node

# Terminal 2: launch RViz display
ros2 launch vector_description display.launch.py

# Terminal 3: open RViz
rviz2
```
### RViz Instructions 
In RViz UI, set **Fixed Frame** to `/odom`.
Add **RobotModel** and set the description topic to `/robot_description`.
To see raw camera images:
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/camera/image_raw` in the top-left dropdown of the UI.

### Demo Driving 
Make Vector drive in a square:
```bash
ros2 run vector_driver drive_square
```

## Acknowledgements

Much inspiration was taken from [nilseuropa/vector_ros2](https://github.com/nilseuropa/vector_ros2/tree/main).  
Thanks to his work, I was able to get a realistic visualization of the Vector in RViz and understand how to approach this project.  
I adapted the node implementations to fit my own project needs.





