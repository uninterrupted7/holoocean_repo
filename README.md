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
                            ──► TF: world → base_link → sonar_link

ROS2 ──► /holoocean/cmd_vel  [geometry_msgs/Twist]  ──► SurfaceVessel thrusters
```

---

## Prerequisites

### 1. Ubuntu 22.04 + ROS2 Humble
```bash
# If not already installed:
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
  ros-humble-geometry-msgs
```

---

## Build & Install

```bash
# 1. Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Copy the package here
cp -r /path/to/holoocean_ros2_bridge .

# 3. Build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holoocean_ros2_bridge
source install/setup.bash
```

---

## Running

### Terminal 1 – Launch bridge (simulation + publishers)
```bash
source ~/ros2_ws/install/setup.bash

# Using the bundled JSON scenario:
ros2 launch holoocean_ros2_bridge bridge.launch.py

# With RViz2:
ros2 launch holoocean_ros2_bridge bridge.launch.py use_rviz:=true

# With a custom scenario file:
ros2 launch holoocean_ros2_bridge bridge.launch.py \
    scenario_file:=/path/to/my_scenario.json

# With real GPS coordinates origin:
ros2 launch holoocean_ros2_bridge bridge.launch.py \
    lat_origin:=37.5665 lon_origin:=126.9780
```

### Terminal 2 – Keyboard teleop
```bash
source ~/ros2_ws/install/setup.bash
ros2 run holoocean_ros2_bridge vessel_teleop
# w/s = forward/back   a/d = turn   SPACE = stop   q = quit
```

### Terminal 3 – Check topics
```bash
ros2 topic list
ros2 topic hz /holoocean/sonar/points   # should see ~5 Hz
ros2 topic hz /holoocean/imu            # should see ~50 Hz
ros2 topic echo /holoocean/gps
```

---

## Sensor → ROS2 Message Reference

| HoloOcean Sensor | Topic | ROS2 Type | Frame |
|------------------|-------|-----------|-------|
| `ProfilingSonar` | `/holoocean/sonar/points` | `sensor_msgs/PointCloud2` | `sonar_link` |
| `ProfilingSonar` | `/holoocean/sonar/image` | `sensor_msgs/Image` (mono8) | `sonar_link` |
| `IMUSensor` | `/holoocean/imu` | `sensor_msgs/Imu` | `imu_link` |
| `GPSSensor` | `/holoocean/gps` | `sensor_msgs/NavSatFix` | `gps_link` |
| `DVLSensor` | `/holoocean/dvl/velocity` | `geometry_msgs/TwistStamped` | `base_link` |
| `PoseSensor` | `/holoocean/odom` | `nav_msgs/Odometry` | `world` |
| `PoseSensor` | `/holoocean/pose` | `geometry_msgs/PoseStamped` | `world` |
| TF | — | TF tree | `world→base_link→sonar_link` |

### PointCloud2 field layout (sonar)
| Field | Type | Description |
|-------|------|-------------|
| `x` | float32 | along-track (vessel forward) |
| `y` | float32 | across-track (port←→starboard) |
| `z` | float32 | depth (positive downward) |
| `intensity` | float32 | sonar return intensity 0–1 |

---

## ProfilingSonar Configuration

The scenario JSON configures the sonar. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Azimuth` | 120° | Total swath width angle |
| `Elevation` | 1° | Along-track beam width |
| `RangeMin` | 0.5 m | Minimum measurable range |
| `RangeMax` | 50.0 m | Maximum measurable range |
| `RangeBins` | 256 | Range resolution |
| `AzimuthBins` | 128 | Number of beams across swath |
| `AddSigma` | 0.05 | Additive noise |
| `MultSigma` | 0.05 | Multiplicative noise |

Increase `RangeBins`/`AzimuthBins` for higher resolution (at compute cost).

---

## Mapping Tools

Once you have data flowing as ROS2 messages, you can use these tools:

### Option A: OctoMap (3D occupancy map from PointCloud2)
```bash
sudo apt install ros-humble-octomap-server ros-humble-octomap-rviz-plugins
ros2 run octomap_server octomap_server_node \
    --ros-args \
    -p frame_id:=world \
    -p resolution:=0.5 \
    -r cloud_in:=/holoocean/sonar/points
```

### Option B: RTABMap (full SLAM with PointCloud2 + Odometry)
```bash
sudo apt install ros-humble-rtabmap-ros
ros2 launch rtabmap_ros rtabmap.launch.py \
    subscribe_depth:=false \
    subscribe_scan_cloud:=true \
    scan_cloud_topic:=/holoocean/sonar/points \
    odom_topic:=/holoocean/odom \
    frame_id:=base_link
```

### Option C: Simple PointCloud accumulation (record a bag and replay)
```bash
# Record
ros2 bag record /holoocean/sonar/points /holoocean/odom /holoocean/gps

# Replay for offline mapping
ros2 bag play <bag_folder>
```

---

## Coordinate Frames

```
world (fixed ENU frame)
  └── base_link (vessel body, from PoseSensor 4×4 matrix)
        └── sonar_link (sonar head, 30 cm below keel, pointing down)
        └── imu_link   (IMU, co-located with base_link)
        └── gps_link   (GPS, co-located with base_link)
```

**HoloOcean uses NED internally.** The bridge publishes in standard ROS ENU.
If your mapping tool expects ENU and you see orientation issues, you may need
to add an ENU↔NED rotation in the TF chain.

---

## Customising the Scenario

Edit `config/surface_mapping_scenario.json` to change:
- World (`"world"`: e.g., `"PierHarbor"`, `"OpenWater"`)
- Agent start position (`"location": [x, y, z]`)
- Sonar parameters (range, azimuth width, noise)
- Add more sensor types (e.g., RaycastLIDAR, RGBCamera)

---

## Troubleshooting

**HoloOcean env fails to start:**
```bash
python -c "import holoocean; print(holoocean.__version__)"
# Should print 2.3.0
python -c "import holoocean; holoocean.install('Ocean')"
```

**No sonar points published:**
- Check `sonar_intensity_threshold` parameter (lower it, e.g., `0.01`)
- Verify sonar socket exists on the agent in the scenario
- The octree needs to initialise (may take a few ticks)

**TF errors in RViz:**
- Ensure `bridge.launch.py` is running (it publishes TF)
- Check `ros2 run tf2_tools view_frames`

**Low sonar frame rate:**
- Reduce `RangeBins` and `AzimuthBins` in the scenario JSON
- Reduce `InitOctreeRange`
