# FDPO ROS1 Stack (Noetic)

ROS1 (Noetic) stack for local navigation with an **HLDS HLS-LFCD2** LiDAR, beacon-based localization (w/ EKF), a navigation controller, and a Stage simulator.  
All configuration and launch wrappers live in the **`conf/`** package (single source of truth).

**Context:** developed for the **Robot Factory 4.0** competition at the **Festival Nacional de Robótica 2025**.

---

## Quick Start

### Build
```bash
cd ~/catkin_ws_fdpo
catkin_make
source devel/setup.bash
```

### Simulation (Stage)
Runs the Stage world and includes navigation + HMI from `conf/`.
```bash
roslaunch simulation_stage run_sim_stage.launch
```

### Real Robot (Full Bring-up)
Brings up hardware (HAL), navigation, and HMI via `conf/`.
```bash
roslaunch conf/script/wake_up_fdpo.launch
```


