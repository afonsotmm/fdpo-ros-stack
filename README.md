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

>  **Important:** run `source devel/setup.bash` in **every terminal** where you run ROS nodes.

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
  * **Raspberry Pi Pico** handling actuator control (low‑level)

> Tip: install missing packages via `apt` (e.g. `sudo apt install ros-noetic-tf2-ros ros-noetic-laser-geometry ros-noetic-dynamic-reconfigure ros-noetic-stage-ros`).

---

## Repository Structure

```
FDPO4.0_2025/
├─ conf/                             # Centralized launch wrappers & parameters
│  └─ script/wake_up_fdpo.launch     # Full bring-up of the real robot
├─ drivers_stack/                    # Hardware drivers
│  └─ hls_lfcd_lds_driver/           # HLDS LiDAR driver (LDS-01/02)
├─ navigation_stack/                 # Localization and navigation controller
│  ├─ localizer/                     # Beacon-based localization with EKF
│  │  ├─ msg/                        # Custom message definitions
│  │  ├─ launch/                     # Launch files for detector and EKF
│  │  ├─ src/                        # Source code (detector + EKF)
│  │  └─ include/                    # Headers
│  └─ navigation_controller/         # Path following and FSM control
│     ├─ cfg/Navigation.cfg          # Dynamic reconfigure parameters
│     ├─ srv/NavigationControl.srv   # Service for controller commands
│     ├─ launch/                     # Controller launch files
│     └─ src/                        # FSM and controller implementation
├─ simulation_stage/                 # Stage-based simulation environment
│  ├─ worlds/                        # Factory floor world file(s)
│  ├─ models/                        # Robot and environment models
│  ├─ launch/                        # Simulation launch files
│  └─ hmi/                           # RViz configs for simulation
├─ utilities_stack/                  # Utility tools
│  └─ path_log/                      # Odometry-to-path logging node
├─ CMakeLists.txt                    # Top-level build system
└─ README.md                         # Project documentation
```

---

## System Overview (Data Flow)

**HLDS LiDAR** → **Beacon Detector (DBSCAN + matching)** → **EKF Localizer** → `odom_filtered` → **Navigation Controller (FSM + gains)** → `cmd_vel` → robot base.

High‑level logic runs on the Raspberry Pi 4 (ROS), while the Raspberry Pi Pico handles motor and actuator control.

Common frames: `map`, `odom`, `base_link`. The controller uses TF to convert goals/paths from `map` → `odom`.

---

## Work in Progress (WIP)

This project is still under active development:

* A high‑level planner for trajectory generation is missing (to be integrated).
* A driver node for communication between the Raspberry Pi 4 and the Raspberry Pi Pico is planned.

---

## Package‑by‑Package Details

### 1) `conf/`

Centralized launch wrappers and parameter files. Instead of launching packages individually, `conf/` provides unified entry points:

* `wake_up_fdpo.launch`: full system bring‑up (drivers + localization + controller + HMI).
* Other YAMLs and launch files unify parameter namespaces.

**Role:** ensures consistency between real robot and simulation.

---

### 2) `drivers_stack/hls_lfcd_lds_driver`

Custom integration of the HLDS LiDAR driver (LDS‑01/02). Handles serial communication, publishes `sensor_msgs/LaserScan`.

**Inputs:** serial data from LiDAR.  
**Outputs:** `/scan` topic.

**Importance:** entry point for environment perception.

---

### 3) `navigation_stack/localizer`

The localizer stack provides LiDAR‑based beacon detection and EKF fusion. It is composed of two nodes: **`beacon_detector_node`** and **`localizer_node` (EKF)**, and three custom messages.

#### a) Custom Messages

* **`localizer/Pose`**: `(float32 x, float32 y)` in the node's frame.
* **`localizer/Cluster`**: `string beacon_match_name`, `Pose[] points`, `Pose centroid`, `uint32 num_points`.
* **`localizer/BeaconMatch`**: `std_msgs/Header header`, `Cluster[] clusters`.

#### b) `beacon_detector_node`

* **Subscribe:** `/base_scan` (`sensor_msgs/LaserScan`).  
* **Publish:**
  * `beacon_estimation` (`localizer/BeaconMatch`) — beacon detection and matching results.
  * `dbscan_markers` (`visualization_msgs/MarkerArray`) — raw clustered points + centroids.
  * `beacons_map_markers` (`visualization_msgs/MarkerArray`, latched) — fixed beacons in robot frame.

**Processing pipeline:**
1. Convert `LaserScan` → 2D points.  
2. **DBSCAN** clustering with `eps` and `minPoints`.  
3. **Nearest‑neighbour matching** between cluster centroids and beacon map.  
4. **Centroid bias compensation** (see equations below).  

---

### Centroid Correction — Equations

(1)  

$$
s = \frac{L}{2R}, \quad \alpha = \arcsin(\mathrm{clip}(s, -1, 1)), \quad L = |p_1 - p_n|
$$

(2)  

$$
\bar{d}_c = \frac{1}{2}\left(|p_c - p_1| + |p_c - p_n|\right)
$$

(3)  

$$
\mathrm{rad} = R^2 \cos^2 \alpha - \left(R^2 - \bar{d}_c^2\right)
$$

(4)  

$$
\Delta r = R \cos \alpha + \sqrt{\max(\mathrm{rad}, 0)}
$$

With $p_{meas}$ being the measured beacon position and  
$\hat{u}_r$ the unit radial vector:

(5)  

$$
p_{\text{corr}} = p_{\text{meas}} + \Delta r \cdot \hat{u}_r, \quad 
\hat{u}_r = \frac{p_{\text{meas}}}{|p_{\text{meas}}|}
$$

**Notes on centroid correction:**  
* Equation (1)–(2): compute arc span and raw centroid distances.  
* Equation (3)–(4): correct radial bias assuming cylindrical beacons of radius *R*.  
* Equation (5): apply correction along the radial unit vector.  
* Parameters `eps`, `minPoints`, `max_match_dist`, and beacon `radius` strongly affect matching performance.  
* Works best when beacons are observed as partial arcs in the LiDAR scan.  
* Incorrect radius values or large occlusions can lead to over- or under-correction.  

---

#### c) `localizer_node` (EKF)

* **Subscribe:**
  * `/odom` (`nav_msgs/Odometry`) → prediction step.
  * `/beacon_estimation` (`localizer/BeaconMatch`) → update step.  
* **State:** $X = [x, y, \theta]$, with covariance $P \in \mathbb{R}^{3 \times 3}$.  
* **Prediction:** motion model with noise.  
* **Measurement model:**

$$
h(X) =
\begin{bmatrix}
r \\
\beta
\end{bmatrix},
\quad
r = \sqrt{(x_b - x)^2 + (y_b - y)^2},
\quad
\beta = \mathrm{atan2}(y_b - y,\, x_b - x) - \theta
$$



* **Main EKF Equations:**

Prediction:  

$$
X_k^- = f(X_{k-1}, u_k)
$$

Covariance prediction:

$$
P_k^- = F_k P_{k-1} F_k^T + Q_k
$$


Kalman gain:  

$$
K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R)^{-1}
$$


State update:  

$$
X_k = X_k^- + K_k (z_k - h(X_k^-))
$$


Covariance update:  

$$
P_k = (I - K_k H_k) P_k^-
$$

* **Outputs:** `odom_filtered` and TF `map → odom`.

---

### 4) `navigation_stack/navigation_controller`

Implements two FSM layers and exposes a service API for control.

#### a) Data structures

* **WayPoint:** `{id, pose{x,y,θ}, align, backwards}`  
* **Dynamic Reconfigure:** `v_nom, w_nom, kp_linear, kp_angular, arrive_radius, yaw_tol`  

#### b) High‑level FSM

Service: `~/control` (`NavigationControl.srv`).  
`command ∈ {start, pause, unpause, stop}`.

#### c) Low‑level FSM (GoToXYθ)

States: `idle → driveToGoal → turnToFinalYaw → done`.



#### Control Laws

Errors:  

$$
e_p =
\begin{bmatrix}
x_d - x \\\\
y_d - y
\end{bmatrix},
\quad
e_\theta = \mathrm{wrap}(\theta_d - \theta)
$$

Angular velocity:  

$$
\omega = \mathrm{clip}(k_\theta e_\theta, -\omega_{nom}, +\omega_{nom})
$$

Linear velocity:  

$$
v = \sigma \cdot \min(k_p \, |e_p|, v_{nom}), \quad \sigma \in \{ +1, -1 \}
$$

Yaw gate (optional):  

$$
g(e_\theta) = \max(0, 1 - |e_\theta| / \theta_{gate})
$$

Arrival conditions:  

$$
|p_d - p_c| \le r_{arrive}, \quad |\theta_d - \theta_c| \le \theta_{tol}
$$

---

### 5) `simulation_stage`

* `factory_floor.world`: Stage world map  
* `run_sim_stage.launch`: launches Stage + stack  

---

### 6) `utilities_stack/path_log`

Node `odoms_to_paths_node` converts odometry into `nav_msgs/Path` for RViz visualization.

---

## Tutorials

A) **Full Simulation**  
```bash
roslaunch simulation_stage run_sim_stage.launch
```

B) **Beacon Localization Only**  
```bash
roslaunch localizer run_beacon_detector.launch
```

C) **EKF Localizer Only**  
```bash
roslaunch localizer run_ekf_localizer.launch
```

D) **Navigation Controller Only**  
```bash
roslaunch navigation_controller run_navigation_controller.launch
```

E) **Real Robot Bring‑up**  
```bash
roslaunch conf/script/wake_up_fdpo.launch
```

---

## Debugging & Troubleshooting

* `rqt_graph`, `rqt_reconfigure`, `rostopic echo`  
* **Problems:** missing `source`, Stage paths, LiDAR serial, TF conflicts  

---

## License

Not defined yet — confirm before external use.

---

## Credits

Developed by **Electrical and Computer Engineering students at FEUP students at FEUP** for **Robot Factory 4.0** (National Robotics Festival 2025):

* *Afonso Mateus*  
* *Christian Geyer*  
* *Daniel Silva*  
* *Pedro Lopes*  
