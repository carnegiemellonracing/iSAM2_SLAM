Overview
=========

What is Path Planning in charge of?
-----------------------------------
Path planning is responsible for determining the route the car should follow to navigate the track. It has two main responsibilities:

1. **Midline Calculation**

2. **SLAM (Simultaneous Localization and Mapping)**

What data does it receive?
---------------------------
Path planning receives cone detections (positions of blue and yellow cones) from the perceptions pipeline. These cone positions are observed using LiDAR and processed in real-time. It also recieves velocity, orientation, and position in the form of twist, quaternion, and pose. These readings are filtered and come from the IMU and GPS. 

.. note:: 
   - Twist gives us linear and angular velocity
   - Quaternion tells us yaw, pitch, and roll
   - Pose gives us position and orientation
   These components combined allow us to reason about the motion of the car as well as the relative position of cones. 

What does it output, and to what system?
-----------------------------------------
The output is a set of path waypoints (from midline calculation) and a SLAM-generated map. The path is sent to the controls pipeline, which uses it to generate control actions for the car. The map is used in later laps to improve navigation. 

.. toctree::
   :maxdepth: 4
   :caption: Contents:

   overview
   implementation
   explainers
   extraneous

