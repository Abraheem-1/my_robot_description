# my_robot_description

A ROS2 differential drive robot with SLAM and Nav2 navigation.

## Requirements
- ROS2 Humble
- Gazebo Classic 11

## Installation

### 1. Clone the repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/my_robot_description.git
```

### 2. Install all dependencies automatically
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Usage

### SLAM Mapping Mode (build a new map)
```bash
ros2 launch my_robot_description slam_and_nav2.launch.py
```
Drive the robot around to build the map:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Save the map when done:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_robot_description/maps/my_map
```

### Navigation Mode (use saved map)
```bash
ros2 launch my_robot_description slam_and_nav2.launch.py use_slam:=false
```
Then use RViz2 Nav2 Goal button to send navigation goals.

## Package Structure
my_robot_description/
├── config/
│   ├── nav2_params.yaml      # Nav2 tuning parameters
│   ├── slam_params.yaml      # SLAM toolbox parameters
│   └── display.rviz          # RViz2 configuration
├── launch/
│   ├── gazebo.launch.py      # Gazebo + robot spawner
│   └── slam_and_nav2.launch.py  # Main launch file
├── maps/
│   ├── my_map.pgm            # Saved occupancy grid map
│   └── my_map.yaml           # Map metadata
├── meshes/                   # Robot STL mesh files
├── urdf/
│   ├── my_robot.xacro        # Robot description
│   ├── my_robot.gazebo       # Gazebo plugins
│   └── materials.xacro       # Visual materials
└── worlds/
    └── my_saved_world.world       
    └── small_warehouse.world       # Simulation environment
    └── warehouse.sdf      
    └── warehouse.world      

## Tuning
See comments inside `config/nav2_params.yaml` and `config/slam_params.yaml`
for detailed explanation of every parameter and when to change it.
