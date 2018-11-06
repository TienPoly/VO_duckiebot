# VO_duckiebot

## Installation
  * Pre-Install: git, catkin, ROS
  * Workspace
      ```
      $ mkdir -p project_VO_ws/src
      $ cd project_VO_ws
      $ catkin init
      ```
  * This repo: our work
      ```
      $ cd project_VO_ws/src
      $ git clone https://github.com/TienPoly/VO_duckiebot.git
      ```
  * viso2: direct method
      ```
      $ cd project_VO_ws/src
      $ git clone https://github.com/TienPoly/viso2.git
      $ cd ..
      $ catkin build viso2
      ```
  * deep learning: to do!
  * Vicon (optional)
      * Ros interface (configured at  [MRASL](https://mrasl.gitbooks.io/documentation/content/) of Polytechnique Montreal) from: https://github.com/MRASL/ros_vrpn_client.git
      * Depedencies
        * vrpn_catkin package from: https://github.com/ethz-asl/vrpn_catkin
        * catkin_simple package from: https://github.com/catkin/catkin_simple.git
        * glog_catkin package from: https://github.com/ethz-asl/glog_catkin.git
      
## Data collection

## Deep learning
