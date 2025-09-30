# FDPO 4.0 (ROS Noetic)

ROS1 (Noetic) stack for local navigation with HLDS HLS‑LFCD2 LiDAR, beacon‑based localization (with EKF), navigation controller, and **Stage** simulation. The **`conf/`** package acts as the *single source of truth* for launch wrappers and parameter configuration.

> Context: developed by **Electrical and Computer Engineering students at FEUP (Faculty of Engineering, University of Porto)** for the **Robot Factory 4.0** competition at the **National Robotics Festival 2025**.

---

## TL;DR (Quick Start)

### Build

```bash
cd ~/catkin_ws_fdpo
catkin_make
source devel/setup.bash
```

### Simulation (Stage)

Runs the Stage world, including navigation + HMI via `conf/`.

```bash
roslaunch simulation_stage run_sim_stage.launch
```

### Real Robot (Full Bring‑up)

Brings up drivers, navigation, and HMI through `conf/`.

```bash
roslaunch conf/script/wake_up_fdpo.launch
```

> **Important:** run `source devel/setup.bash` in **every terminal** where you run ROS nodes.

---

## Requirements

* **OS:** Ubuntu 20.04 LTS (recommended for ROS Noetic)
* **ROS:** ROS1 Noetic (with `roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`)
* **Dependencies:**

  * `tf2`, `tf2_ros`, `tf2_geometry_msgs`
  * `laser_geometry`
  * `message_generation`, `message_runtime`
  * `dynamic_reconfigure`
  * Visualization: `rviz`, `visualization_msgs`
* **Simulation:** Stage (`stage_ros`)
* **Hardware (real robot):**

  * HLDS HLS‑LFCD LiDAR (LDS‑01/02) — *driver included in this repository*.
  * **Raspberry Pi 4** running ROS and this stack (high‑level)
  * **Raspberry Pi Pico** handling motor and actuator control (low‑level)

> Tip: install missing packages via `apt` (e.g. `sudo apt install ros-noetic-tf2-ros ros-noetic-laser-geometry ros-noetic-dynamic-reconfigure ros-noetic-stage-ros`).

---

## Repository Structure

```
FDPO4.0_2025/
├─ conf/                      # Launch wrappers & params (single source of truth)
│  └─ script/wake_up_fdpo.launch
├─ drivers_stack/
│  └─ hls_lfcd_lds_driver/    # HLDS LiDAR driver (LDS‑01/02)
├─ navigation_stack/
│  ├─ localizer/              # Beacon‑based localization + EKF
│  │  ├─ msg/                 # Pose.msg, Cluster.msg, BeaconMatch.msg
│  │  ├─ launch/
│  │  │  ├─ beacon_detector/run_beacon_detector.launch
│  │  │  └─ ekf_localizer/run_ekf_localizer.launch
│  │  ├─ src/
│  │  │  ├─ beacon_detector/  # DBSCAN clustering + beacon matching
│  │  │  └─ ekf_localizer/    # EKF fusion
│  │  └─ include/
│  └─ navigation_controller/  # Controller + FSM + dynamic_reconfigure
│     ├─ cfg/Navigation.cfg   # Runtime parameters
│     ├─ srv/NavigationControl.srv
│     ├─ launch/run_navigation_controller.launch
│     └─ src/{fsm.cpp, navigation_controller_node.cpp}
├─ simulation_stage/          # Stage worlds/models/launch
│  ├─ worlds/factory_floor.world
│  ├─ models/
│  ├─ launch/run_sim_stage.launch
│  └─ hmi/rviz/fdpo_simulation.rviz (optional)
├─ utilities_stack/
│  └─ path_log/               # Logging/visualization utilities
│     └─ src/odoms_to_paths_node.cpp
├─ CMakeLists.txt (top‑level)
└─ README.md
```

---

## System Overview (Data Flow)

**HLDS LiDAR** → **Beacon Detector (DBSCAN + matching)** → **EKF Localizer** → `odom_filtered` → **Navigation Controller (FSM + gains)** → `cmd_vel` → robot base.

*High‑level logic runs on the Raspberry Pi 4 (ROS), while the Raspberry Pi Pico handles motor and actuator control.*

Common frames: `map`, `odom`, `base_link`. The controller uses TF to convert goals/paths from `map` → `odom`.

---

## Work in Progress (WIP)

This project is still under active development:

* A **high‑level planner** for trajectory generation is missing (to be integrated).
* A **driver node** for communication between the Raspberry Pi 4 and the Raspberry Pi Pico is planned.

---

## Packages & Nodes

### 1) `drivers_stack/hls_lfcd_lds_driver`

Driver for HLDS LiDAR (LDS‑01/02). Publishes `sensor_msgs/LaserScan` (e.g. `/scan`), with `frame_id` set to the sensor.

**Suggested setup**

* Check USB/serial port (`/dev/ttyUSB*`, add `udev` rule if needed).
* Verify `frame_id` and frequency (10 Hz typical).

**Example launch**

```bash
roslaunch hls_lfcd_lds_driver hlds_laser.launch
```

> This repo already includes the driver; use its launch files or the wrappers in `conf/`.

---

### 2) `navigation_stack/localizer`

Beacon‑based localization with EKF. Defines custom messages:

* `Pose.msg`: 2D pose (x, y, yaw)
* `Cluster.msg`: result of DBSCAN clustering
* `BeaconMatch.msg`: matched beacon↔cluster pair

#### 2.1) **Beacon Detector** (`beacon_detector_node`)

Clustering of LiDAR points (DBSCAN), projection to robot frame, beacon matching.

**Subscribed topics**

* `sensor_msgs/LaserScan` (e.g. `/base_scan`)

**Published topics**

* `localizer/BeaconMatch` → `/beacon_Estimation`
* `visualization_msgs/MarkerArray` → `/dbscan_markers`, `/beacons_map_markers`

**Key parameters**

* **Beacon map** (static positions in `map` frame)
* DBSCAN: `eps`, `minPts`
* `maxMatchDist`: maximum radius for beacon↔cluster association

**Launch**

```bash
roslaunch localizer run_beacon_detector.launch
```

#### 2.2) **EKF Localizer** (`localizer_node`)

EKF fusion of odometry + beacon detections. Outputs filtered odometry.

**Subscribed topics**

* Base odometry (`nav_msgs/Odometry`)
* `localizer/BeaconMatch`

**Published topics**

* Filtered odometry (`nav_msgs/Odometry`)
* TF `map↔odom` (or `odom↔base_link`)

**Launch**

```bash
roslaunch localizer run_ekf_localizer.launch
```

---

### 3) `navigation_stack/navigation_controller`

Navigation controller (FSM) with **Dynamic Reconfigure** and service interface.

**Interfaces**

* Service `~/control` (`NavigationControl.srv`): modes (`idle`/`start`/`pause`/`stop`)
* Dynamic Reconfigure: adjust gains/thresholds live via `rqt_reconfigure`

**Topics**

* Publishes velocity commands (`geometry_msgs/Twist`) → `cmd_vel`
* Subscribes to goals/paths (from RViz, service, or params)

**Launch**

```bash
roslaunch navigation_controller run_navigation_controller.launch
```

**Runtime tuning**

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

**Service call example**

```bash
rosservice call /navigation_controller/control "<NavigationControl fields>"
```

---

### 4) `utilities_stack/path_log`

Utility to convert `nav_msgs/Odometry` to `nav_msgs/Path` for visualization/debug.

**Example**

```bash
rosrun path_log odoms_to_paths_node
```

---

### 5) `simulation_stage`

Stage world + launch wrappers.

**Launch**

```bash
roslaunch simulation_stage run_sim_stage.launch
```

**Contents**

* `worlds/factory_floor.world`
* `models/`
* Optional RViz config: `hmi/rviz/fdpo_simulation.rviz`

---

## Tutorials (Step‑by‑Step)

### A) Full Simulation (Stage + navigation + HMI)

1. Build & source workspace:

   ```bash
   cd ~/catkin_ws_fdpo && catkin_make && source devel/setup.bash
   ```
2. Run simulation:

   ```bash
   roslaunch simulation_stage run_sim_stage.launch
   ```
3. (Optional) Open RViz with pre‑configured layout:

   ```bash
   rviz -d $(rospack find simulation_stage)/hmi/rviz/fdpo_simulation.rviz
   ```
4. Send navigation goals in RViz. If `rviz_append:=true`, each click adds to the route.
5. Adjust controller gains live with `rqt_reconfigure`.

### B) Beacon Localization Only (debug in RViz)

1. Ensure LiDAR publishing `LaserScan`.
2. Start beacon detector:

   ```bash
   roslaunch localizer run_beacon_detector.launch
   ```
3. In RViz, visualize `/dbscan_markers`, `/beacons_map_markers`, and `LaserScan`.

### C) EKF + Filtered Odometry

1. Ensure odometry + beacon detections are available.
2. Start EKF:

   ```bash
   roslaunch localizer run_ekf_localizer.launch
   ```
3. Inspect `odom_filtered` vs `odom` in RViz.

### D) Navigation Controller Only

1. Configure parameters (e.g. in `navigation_parameters.yaml`).
2. Launch controller:

   ```bash
   roslaunch navigation_controller run_navigation_controller.launch
   ```
3. Send goals from RViz or via `~/control` service.
4. Check published `cmd_vel`.

### E) Real Robot Bring‑up

1. Connect HLDS LiDAR (check USB port).
2. Build & source workspace.
3. Launch everything:

   ```bash
   roslaunch conf/script/wake_up_fdpo.launch
   ```
4. Open RViz and check TF, odometry, and velocity commands.

---

## Parameters (Summary)

* **Beacon Detector**: beacon map, DBSCAN `eps`/`minPts`, `maxMatchDist`
* **EKF Localizer**: process/measurement noise, input topics, frame conventions
* **Navigation Controller**: loop rate, gains, tolerances, append flag
* **Stage Simulation**: world, models, topic remaps

---

## Debugging Tips

* `rqt_graph` → visualize node/topic graph
* `rqt_reconfigure` → tune controller live
* `rostopic echo/Hz` → inspect topics
* `rosbag record` → log sessions (for EKF debugging)
* RViz: enable `TF`, `Odometry`, `Path`

---

## Troubleshooting

* **"Package not found"** → missing `source devel/setup.bash`
* **Stage world not loading** → check `stage_ros` installed and paths
* **`rqt_reconfigure` missing** → install `ros-noetic-dynamic-reconfigure`
* **LiDAR not starting** → check serial port, permissions, baudrate, `udev` rule
* **Conflicting TF** → ensure only one publisher per TF link

---

## License

No explicit license yet in the repository. Confirm before reusing outside this project.

---

## Credits

Project developed by **Electrical and Computer Engineering students at FEUP (Faculty of Engineering, University of Porto)**:

* *Afonso Mateus*
* *Christian Geyer*
* *Daniel Silva*
* *Pedro Lopes*

Contributions and PRs are welcome.
