Overview
=========

What is Path Planning in charge of?
-----------------------------------
Path planning is responsible for determining the route the car should follow to navigate the track. It has two main responsibilities:

1. **Midline Calculation**
2. **SLAM (Simultaneous Localization and Mapping)**

What data does it recieve?
---------------------------
Path planning recieves cone detections (positions of blue and yellow cones) from the perceptions pipeline. These cone positions are observed using LiDAR and processed in real-time. 

What does it output, and to what system?
-----------------------------------------
The output is a set of path waypoints (from midline calculation) and a SLAM-generated map. The path is sent to the controls pipeline, which uses it to generate control actions for the car. The map is used in later laps to improve navigation. 

.. toctree::
   :maxdepth: 2
   :caption: Contents:

