# holoocean_ros2_bridge

ROS2 Humble ↔ HoloOcean v2.3.0 bridge for **seafloor mapping with a surface vessel and multibeam sonar**.

---

## Architecture Overview

```
HoloOcean (Python)
  └── SurfaceVessel agent
        ├── ProfilingSonar  ──► /holoocean/sonar/points  [sensor_msgs/PointCloud2]
        │                  ──► /holoocean/sonar/image   [sensor_msgs/Image]
        ├── IMUSensor       ──► /holoocean/imu           [sensor_msgs/Imu]
        ├── GPSSensor       ──► /holoocean/gps           [sensor_msgs/NavSatFix]
        ├── DVLSensor       ──► /holoocean/dvl/velocity  [geometry_msgs/TwistStamped]
        └── PoseSensor      ──► /holoocean/odom          [nav_msgs/Odometry]
                            ──► /holoocean/pose          [geometry_msgs/PoseStamped]
                            ──► /holoocean/vessel_marker [visualization_msgs/Marker]
                            ──► TF: world → base_link

ROS2 ──► /holoocean/cmd_vel  [geometry_msgs/Twist]  ──► SurfaceVessel thrusters
```

---

## Features

- **Autonomous mode**: Built-in autonomous path pattern (forward → left → backward → right)
- **Sonar visualization**: PointCloud2 in vessel-relative frame (base_link)
- **Vessel markers**: RViz visualization with cube + arrow showing position/heading
- **Multiple sensors**: IMU, GPS, DVL, Pose all bridged to ROS2

---

## Prerequisites

### 1. Ubuntu 22.04 + ROS2 Humble
```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### 2. HoloOcean v2.3.0
```bash
pip install holoocean==2.3.0
python -c "import holoocean; holoocean.install('Ocean')"
```

### 3. ROS2 Python dependencies
```bash
sudo apt install \
  ros-humble-tf2-ros \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-visualization-msgs
```

---

## Build & Install

```bash
# 1. Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone or copy the package
git clone https://github.com/YOUR_USERNAME/holoocean_ros2_bridge.git

# 3. Build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holoocean_ros2_bridge
source install/setup.bash
```

---

## Running

### Launch Options

```bash
source ~/ros2_ws/install/setup.bash

# Basic launch (manual control)
ros2 launch holoocean_ros2_bridge bridge.launch.py

# With autonomous movement (forward 5s → left 5s → backward 5s → right 5s)
ros2 launch holoocean_ros2_bridge bridge.launch.py auto_mode:=true

# With custom scenario file
ros2 launch holoocean_ros2_bridge bridge.launch.py \
    scenario_file:=/path/to/my_scenario.json
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `scenario_file` | `config/surface_mapping_scenario.json` | Path to HoloOcean scenario JSON |
| `auto_mode` | `false` | Enable autonomous movement pattern |
| `thrust_scale` | `50.0` | Thrust multiplier for vessel control |
| `sonar_intensity_threshold` | `0.1` | Minimum intensity for sonar points |

### Keyboard Teleop (when auto_mode:=false)

```bash
ros2 run holoocean_ros2_bridge vessel_teleop
```

**Controls:**
- `w` - Toggle forward
- `s` - Toggle backward
- `a` - Toggle left turn
- `d` - Toggle right turn
- `SPACE` - Stop
- `q` - Quit

---

## Topics

| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/holoocean/sonar/points` | `sensor_msgs/PointCloud2` | `base_link` | Sonar points (relative to vessel) |
| `/holoocean/sonar/image` | `sensor_msgs/Image` | `base_link` | Raw sonar intensity |
| `/holoocean/imu` | `sensor_msgs/Imu` | `imu_link` | IMU data |
| `/holoocean/gps` | `sensor_msgs/NavSatFix` | `gps_link` | GPS coordinates |
| `/holoocean/dvl/velocity` | `geometry_msgs/TwistStamped` | `base_link` | DVL velocity |
| `/holoocean/odom` | `nav_msgs/Odometry` | `world` | Ground truth pose |
| `/holoocean/pose` | `geometry_msgs/PoseStamped` | `world` | Ground truth pose |
| `/holoocean/vessel_marker` | `visualization_msgs/Marker` | `base_link` | Vessel visual |

### PointCloud2 Format

| Field | Type | Description |
|-------|------|-------------|
| `x` | float32 | Across-track (starboard positive) |
| `y` | float32 | Along-track (forward positive) |
| `z` | float32 | Depth (negative = below vessel in ENU) |
| `intensity` | float32 | Sonar return intensity 0–1 |

---

## RViz Visualization

```bash
# Launch with RViz
ros2 launch holoocean_ros2_bridge bridge.launch.py use_rviz:=true

# Or run RViz separately
rviz2 -d ~/ros2_ws/src/holoocean_ros2_bridge/rviz/mapping.rviz
```

The RViz config sets:
- Fixed frame: `base_link`
- Grid on XZ plane
- Sonar PointCloud with intensity coloring
- Vessel markers (cube + arrow)

---

## Configuration

Edit `config/surface_mapping_scenario.json`:

```json
{
    "name": "SurfaceVessel-Mapping",
    "world": "PierHarbor",
    "ticks_per_sec": 30,
    "agents": [{
        "agent_type": "SurfaceVessel",
        "location": [0.0, 0.0, -1.0],
        "sensors": [
            {"sensor_type": "ProfilingSonar", "Hz": 10, ...},
            {"sensor_type": "IMUSensor", "Hz": 30, ...},
            {"sensor_type": "GPSSensor", "Hz": 5, ...},
            {"sensor_type": "DVLSensor", "Hz": 10, ...},
            {"sensor_type": "PoseSensor", "Hz": 30}
        ]
    }]
}
```

---

## Mapping Tools

### OctoMap
```bash
sudo apt install ros-humble-octomap-server ros-humble-octomap-rviz-plugins
ros2 run octomap_server octomap_server_node \
    -p frame_id:=base_link \
    -p resolution:=0.5 \
    -r cloud_in:=/holoocean/sonar/points
```

### RTABMap
```bash
sudo apt install ros-humble-rtabmap-ros
ros2 launch rtabmap_ros rtabmap.launch.py \
    subscribe_depth:=false \
    subscribe_scan_cloud:=true \
    scan_cloud_topic:=/holoocean/sonar/points \
    odom_topic:=/holoocean/odom \
    frame_id:=base_link
```

### ROS Bag
```bash
ros2 bag record /holoocean/sonar/points /holoocean/odom /holoocean/gps
ros2 bag play <bag_folder>
```

---

## Troubleshooting

**HoloOcean env fails to start:**
```bash
python -c "import holoocean; print(holoocean.__version__)"
python -c "import holoocean; holoocean.install('Ocean')"
```

**Vessel not moving:**
- Check `thrust_scale` parameter (default 50.0)
- Use `auto_mode:=true` to test autonomous movement

**No sonar points:**
- Vessel must be underwater (z < 0)
- Check `sonar_intensity_threshold` (try 0.01)

**TF errors in RViz:**
- Ensure launch is running
- Check `ros2 run tf2_tools view_frames`