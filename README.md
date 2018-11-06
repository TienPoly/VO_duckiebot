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
  * Direct method VO: viso2 package
      ```
      $ cd project_VO_ws/src
      $ git clone https://github.com/TienPoly/viso2.git
      ```
  * Ground projection: to do!
      https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/ground_projection

  * Deep learning: to do!
  * Vicon (optional)
      * [Vicon motion capture system overview](https://mrasl.gitbooks.io/documentation/vicon.html)
      * Ros interface (configured at  [MRASL](https://mrasl.gitbooks.io/documentation/content/) of Polytechnique Montreal) from: https://github.com/MRASL/ros_vrpn_client.git
      * Depedencies
        * vrpn_catkin package from: https://github.com/ethz-asl/vrpn_catkin
        * catkin_simple package from: https://github.com/catkin/catkin_simple.git
        * glog_catkin package from: https://github.com/ethz-asl/glog_catkin.git
  * Build
      ```
      $ cd project_VO_ws
      $ catkin build  
      ```

## Data collection

## Deep learning
