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
![eq1](https://latex.codecogs.com/png.latex?bg=whites%20%3D%20%5Cfrac%7BL%7D%7B2R%7D%2C%20%5Cquad%20%5Calpha%20%3D%20%5Carcsin%28%5Coperatorname%7Bclip%7D%28s%2C-1%2C1%29%29%2C%20%5Cquad%20L%20%3D%20%7C%20p_1%20-%20p_n%20%7C.)

(2)  
![eq2](https://latex.codecogs.com/png.latex?bg=white%5Cbar%7Bd%7D_c%20%3D%20%5Ctfrac%7B1%7D%7B2%7D%5Cleft%28%20%7C%20p_c%20-%20p_1%20%7C%20%2B%20%7C%20p_c%20-%20p_n%20%7C%20%5Cright%29.)

(3)  
![eq3](https://latex.codecogs.com/png.latex?bg=white%5Cmathrm%7Brad%7D%20%3D%20R%5E2%5Ccos%5E2%5Calpha%20-%20%5Cleft%28%20R%5E2%20-%20%5Cbar%7Bd%7D_c%5E2%20%5Cright%29.)

(4)  
![eq4](https://latex.codecogs.com/png.latex?bg=white%5CDelta%20r%20%3D%20R%5Ccos%5Calpha%20%2B%20%5Csqrt%7B%5Cmax%28%5Cmathrm%7Brad%7D%2C%200%29%7D.)

With ![pmeas](https://latex.codecogs.com/png.latex?p_%7B%5Ctext%7Bmeas%7D%7D) being the measured beacon position and  
![urhat](https://latex.codecogs.com/png.latex?%5Chat%7Bu%7D_r%20%3D%20%5Cdfrac%7Bp_%7B%5Ctext%7Bmeas%7D%7D%7D%7B%7C%20p_%7B%5Ctext%7Bmeas%7D%7D%20%7C%7D) the unit radial vector:

(5)  
![eq5](https://latex.codecogs.com/png.latex?bg=whitep_%7B%5Ctext%7Bcorr%7D%7D%20%3D%20p_%7B%5Ctext%7Bmeas%7D%7D%20%2B%20%5CDelta%20r%20%5Ccdot%20%5Chat%7Bu%7D_r.)

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
* **State:** ![X](https://latex.codecogs.com/png.latex?X%20%3D%20%5Bx%2C%20y%2C%20%5Ctheta%5D), with covariance ![P](https://latex.codecogs.com/png.latex?P%20%5Cin%20%5Cmathbb%7BR%7D%5E%7B3%5Ctimes3%7D).  
* **Prediction:** motion model with noise.  
* **Measurement model:**

![hX](https://latex.codecogs.com/png.latex?h%28X%29%20%3D%20%5Cbegin%7Bbmatrix%7D%20r%20%5C%5C%20%5Cbeta%20%5Cend%7Bbmatrix%7D%2C%20%5Cquad%20r%20%3D%20%5Csqrt%7B%28x_b-x%29%5E2%20%2B%20%28y_b-y%29%5E2%7D%2C%20%5Cquad%20%5Cbeta%20%3D%20%5Coperatorname%7Batan2%7D%28y_b-y%2C%20x_b-x%29%20-%20%5Ctheta.)

* **Main EKF Equations:**

Prediction:  
![pred](https://latex.codecogs.com/png.latex?bg=whiteX_k%5E-%20%3D%20f%28X_%7Bk-1%7D%2C%20u_k%29)  

Covariance prediction:  
![covpred](https://latex.codecogs.com/png.latex?bg=whiteP_k%5E-%20%3D%20F_k%20P_%7Bk-1%7D%20F_k%5ET%20%2B%20Q_k)  

Kalman gain:  
![K](https://latex.codecogs.com/png.latex?bg=whiteK_k%20%3D%20P_k%5E-%20H_k%5ET%20%28H_k%20P_k%5E-%20H_k%5ET%20%2B%20R%29%5E-1)  

State update:  
![xupd](https://latex.codecogs.com/png.latex?bg=whiteX_k%20%3D%20X_k%5E-%20%2B%20K_k%20%28z_k%20-%20h%28X_k%5E-%29%29)  

Covariance update:  
![pupd](https://latex.codecogs.com/png.latex?bg=whiteP_k%20%3D%20%28I-K_kH_k%29P_k%5E-)  

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
![e_defs](https://latex.codecogs.com/png.latex?bg=whitee_p%20%3D%20%5B%20x_d-x%2C%20y_d-y%20%5D%5ET%2C%20%5Cquad%20e_%5Ctheta%20%3D%20wrap%28%5Ctheta_d-%5Ctheta%29.)

Angular velocity:  
![wlaw](https://latex.codecogs.com/png.latex?bg=white%5Comega%20%3D%20clip%28k_%5Ctheta%20e_%5Ctheta%2C%20-%5Comega_%7Bnom%7D%2C%20%2B%5Comega_%7Bnom%7D%29.)

Linear velocity:  
![vlaw](https://latex.codecogs.com/png.latex?bg=whitev%20%3D%20%5Csigma%20%5Ccdot%20min%28k_p%5C%7Ce_p%5C%7C%2C%20v_%7Bnom%7D%29%2C%20%5Csigma%20%5Cin%20%5C%7B%2B1%2C-1%5C%7D.)

Yaw gate (optional):  
![gate](https://latex.codecogs.com/png.latex?bg=whiteg%28e_%5Ctheta%29%20%3D%20max%280%2C1-%7Ce_%5Ctheta%7C/%5Ctheta_%7Bgate%7D%29.)

Arrival conditions:  
![arrive](https://latex.codecogs.com/png.latex?bg=white%5C%7Cp_d-p_c%5C%7C%20%5Cle%20r_%7Barrive%7D%2C%20%5Cquad%20%7C%5Ctheta_d-%5Ctheta_c%7C%20%5Cle%20%5Ctheta_%7Btol%7D.)

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
