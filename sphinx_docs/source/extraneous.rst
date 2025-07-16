Extraneous
=================

Vocabulary and Definitions
---------------------------
SLAM: Simultaneous Localization and Mapping, A method used by autonomous systems to build a map of an unknown environment while simultaneously keeping track of the system's own position within that environment.

iSAM2: A specific algorithm for SLAM (Incremental Smoothing and Mapping, version 2). It uses a factor graph representation and performs efficient incremental updates to optimize the vehicle's pose and the map of the environment.

Factor Graph: A bipartite graph consisting of:

- **Variable nodes**: Represent unknowns, such as the vehicle pose or landmark positions.

- **Factor nodes**: Represent probabilistic constraints between variables, such as measurements or priors.

The factor graph encodes the full probabilistic model used for SLAM.

Variable Node: A node in the factor graph representing a variable to be estimated, such as a vehicle pose (`x_n`) or landmark position (`l_n`).

Factor Node: A node that connects one or more variable nodes and encodes a probabilistic constraint, such as a sensor measurement.

Prior Factor: A special type of factor node used to initialize the estimation problem with known values (e.g., the first car pose).

Mahalanobis Distance: A distance metric that accounts for uncertainty in measurements. It measures the number of standard deviations a point is from the mean of a distribution. Used for data association in SLAM.

Data Association: The process of matching observed landmarks (e.g., cones) with previously seen landmarks in the map. Essential for consistent mapping and localization.

Pose: The position and orientation of the vehicle in the global frame. Often represented as (x, y, θ) in 2D SLAM.

Quaternion: A 4D representation of 3D orientation that avoids singularities and discontinuities. Consists of four components: (w, x, y, z).

Twist: A message (in ROS) containing the linear and angular velocity of a vehicle, used for estimating motion.

Odometry: The use of motion sensors (such as encoders, IMU, or GPS) to estimate the position and orientation of a robot over time.

Landmark: A static feature in the environment used for localization, such as a cone in Formula Student Driverless.

ROS (Robot Operating System): An open-source framework for building robotic systems. ROS provides tools, libraries, and conventions for writing modular robot software, including message-passing between processes (nodes), hardware abstraction, and device drivers. In this project, ROS 2 is used to implement and run various nodes such as SLAM, perception, and control.

ROS 2 Node: A process that performs computation in the ROS 2 framework. In this context, SLAM nodes are responsible for managing factor graphs, receiving sensor data, and publishing localization estimates.

TBB (Threading Building Blocks): A C++ library used for parallel programming. It can improve performance in multi-threaded SLAM systems.

Extra Thoughts
-----------------
While the current SLAM and path planning system is designed to be efficient and robust, several challenges may arise in practice:

    Sensor Noise and Drift: GPS and IMU measurements can be noisy or suffer from drift. This can lead to inaccurate localization or erroneous map updates.

    Data Association Ambiguity: Identifying whether a newly observed cone corresponds to a previously seen cone (data association) is challenging in cluttered environments. Misassociations can degrade map quality and cause localization accuracy.

    Tuning Sensitivity: SLAM performance heavily depends on parameters such as the Mahalanobis distance threshold and noise models. Poorly tuned parameters can result in either missed associations or false positives.

    Failure Recovery: If the system becomes mislocalized or the factor graph diverges due to accumulated error, recovering gracefully is non-trivial and may require additional loop closure strategies or reinitialization.

    Real World Edge Cases: Lighting conditions, partial occlusions, or uneven terrain can cause discrepancies between perception and ground truth, which affect downstream SLAM and planning modules.