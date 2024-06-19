======================================
How to run iSAM2 on the EUFS simulator
======================================

Prerequisites
-------------
- Install ROS Foxy
- Install colcon
- Install Ubuntu 20.04 LTS



1.) Clone the git repository
----------------------------
git clone [url-of-repository]

2.) Change directories to the eufs directory clone the necessary repositories
-----------------------------------------------------------------------------
- If the following directories already exist but are empty, make sure to remove them before cloning
- eufs_sim: https://gitlab.com/eufs/eufs_sim.git
- eufs_msgs: https://gitlab.com/eufs/eufs_msgs.git
- eufs_rviz_plugins: https://gitlab.com/eufs/eufs_rviz_plugins.git

3.) Install colcon extensions and set the EUFS_MASTER environment variable
--------------------------------------------------------------------------
`pip3 install colcon-common-extensions -U`
`echo 'export EUFS_MASTER=/path/to/iSAM2_SLAM/eufs' >> ~/.bashrc`
`source ~/.bashrc`

4.) Install dependencies for the EUFS simulator using rosdep
------------------------------------------------------------
`sudo apt-get install python3-rosdep`
`sudo rosdep init`
`rosdep update`
`rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y`

5.) Build the eufs package and source it
----------------------------------------
- Change directories to /path/to/iSAM2_SLAM/eufs
- Run ``colcon build``
- If the directories ``build``, ``log``, or ``install``, currently exist in eufs, make sure to remove them before building
- Run ``source install/setup.bash`` from /path/to/iSAM2_SLAM/eufs

6.) Change directories into /path/to/iSAM2_SLAM/workspace/src and make sure that the eufs_msgs has been built
-------------------------------------------------------------------------------------------------------------
- If the directory exists but is empty, remove the directory, and then clone eufs_msgs: https://gitlab.com/eufs/eufs_msgs.git

7.) Build the iSAM2 package and source it
-----------------------------------------
- Change directories to /path/to/iSAM2_SLAM/workspace
- Run ``colcon build``
- If the directories ``build``, ``log``, or ``install``, currently exist in eufs, make sure to remove them before building
- Run ``source install/setup.bash`` from /path/to/iSAM2_SLAM/workspace

8.) Run the EUFS simulator
--------------------------
- Open start_sim.txt using a text editor
- This file is responsible for running the simulator by first changing directories to the eufs directory.
- Make sure to change this path to the your /path/to/iSAM2_SLAM/eufs
- Save the changes made
- To run the simulator, call ``bash start_sim.txt``

9.) In another terminal run the iSAM2 node
------------------------------------------
- Make sure to build and source the package (from step 5)
- Run ``ros2 run isam2 isam_test``
- isam2 is the name of the package
- isam_test is the executable name
- For more information, see the CMakeLists.txt file

Note: whenever you open a new terminal, you must source the package you plan to run
-----------------------------------------------------------------------------------
- When running the simulator, remember to change directories to /path/to/iSAM2_SLAM/eufs first and run source install/setup.bash before running bash start_sim.txt
- When running the iSAM2, remember to change directories to /path/to/iSAM2_SLAM/workspace first and run source install/setup.bash before running ``ros2 run isam2 isam_test``
