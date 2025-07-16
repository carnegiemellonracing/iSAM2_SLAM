Implementation
=====================

SLAM Nodes
----------------
There are three primary nodes for SLAM. 

1. **real_data_slam_node_gps**

2. **real_data_slam_node_no_gps**

3. **controls_sim_slam_node**

The first two nodes are used to run on real data depending on whether we have GPS or not. The last node is used to test our SLAM implementation in house with a rosbag recorded on our in house controls sim. 


Execution and Data follow
----------------------------

**Where does iSAM2 run (CPU/GPU)?**

The iSAM2 SLAM implementation runs entirely on the CPU. It is written in C++ and uses GTSAM, a CPU-optimized factor graph library. No GPU acceleration is used, as iSAM2 is designed for incremental updates that are efficient enough for real time execution on modern multi-core CPUs. Our SLAM nodes run within a ROS2 C++ node and leverages threading when available (through TBB), although much of the computation remains serial due to the incremental nature of the updates.

**What is the data flow?**

1. Input: 

- Cone observations from the perceptions pipeline (2D positions of blue/yellow cones)

- IMU and velocity estimates

- Pose estimates from GPS (optional)

2. Processing: 

- Each cone observation is added as a bearing-range factor connecting the vehicle pose to cone landmarks

- The current vehicle pose (if available) is added as a new variable in the factor graph 

- Motion priors are added between sequential poses (using IMU/velocity)

- This information is fed into iSAM2 which incrementally updates the estimates of all previous poses and landmark (cone) positions

3. Output: 

- Estimated vehicle pose (2D position and orientation) at each time step

- Updated cone map as global positions of all landmarks (cones)
