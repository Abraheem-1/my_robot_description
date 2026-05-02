# my_robot_description

A ROS2 Humble differential drive robot with SLAM and Nav2 autonomous navigation,
simulated in Gazebo Classic 11 with the AWS small warehouse world.

## Requirements

- ROS2 Humble
- Gazebo Classic 11
- slam_toolbox
- Nav2
- AWS RoboMaker Small Warehouse World

## Installation

### 1. Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Abraheem-1/my_robot_description.git
```

### 2. Install the AWS warehouse world

```bash
sudo apt update
sudo apt install ros-humble-aws-robomaker-small-warehouse-world
```

### 3. Fix Gazebo model URIs (required — apt package has broken paths)

```bash
cp -rL /opt/ros/humble/share/aws_robomaker_small_warehouse_world/models/* ~/.gazebo/models/
grep -rl 'file://models/' ~/.gazebo/models/ | xargs sed -i 's|file://models/|model://|g'
```

### 4. Install all dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Usage

### SLAM Mapping Mode (build a new map)

```bash
ros2 launch my_robot_description slam_and_nav2.launch.py use_slam:=true
```

Drive the robot around the warehouse to build the map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When the map looks complete, save it — **both commands required**:

```bash
# Saves .pgm + .yaml (occupancy grid)
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_robot_description/maps/my_warehouse_map

# Saves .posegraph + .data (required for slam_toolbox localization)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/$USER/ros2_ws/src/my_robot_description/maps/my_warehouse_map'}"
```

### Navigation Mode (use saved map)

```bash
ros2 launch my_robot_description slam_and_nav2.launch.py use_slam:=false
```

Once RViz2 opens (~12 seconds):
1. Click **2D Pose Estimate** and click where the robot spawned on the map
2. Wait for the LiDAR scan to align with the map walls
3. Click **Nav2 Goal** and click a destination on the map

## Robot Specs

| Property | Value |
|---|---|
| Chassis | 0.20 x 0.15 x 0.06 m |
| Wheel radius | 0.04 m |
| Wheel separation | 0.175 m |
| LiDAR range | 0.15 – 10.0 m, 360° |
| Controller | Regulated Pure Pursuit |

## Package Structure
my_robot_description/
├── config/
│   ├── nav2_params.yaml          # Nav2 tuning parameters
│   ├── slam_params.yaml          # SLAM toolbox parameters
│   └── display.rviz              # RViz2 configuration
├── launch/
│   ├── gazebo.launch.py          # Gazebo + robot spawner
│   └── slam_and_nav2.launch.py   # Main launch file
├── maps/
│   ├── my_warehouse_map.pgm      # Occupancy grid map
│   ├── my_warehouse_map.yaml     # Map metadata
│   ├── my_warehouse_map.data     # slam_toolbox pose graph data
│   └── my_warehouse_map.posegraph # slam_toolbox pose graph (required for localization)
├── meshes/                       # Robot STL mesh files
├── urdf/
│   ├── my_robot.xacro            # Robot description
│   ├── my_robot.gazebo           # Gazebo plugins
│   └── materials.xacro           # Visual materials
└── worlds/
└── (custom worlds)           # Gazebo world files

## Known Issues & Fixes

### Warehouse models not loading in Gazebo
The apt package for `aws_robomaker_small_warehouse_world` ships with broken
`model.sdf` files that use `file://models/` URIs instead of the correct
`model://` format. The fix in step 3 of Installation corrects this.

### slam_toolbox localization starts blank instead of loading saved map
Make sure you have saved the map using **both** `map_saver_cli` and
`serialize_map` service. The `.posegraph` file is required for localization
mode — saving with `map_saver_cli` alone is not sufficient.

## Tuning

See comments inside `config/nav2_params.yaml` and `config/slam_params.yaml`
for detailed explanation of every parameter and when to change it.