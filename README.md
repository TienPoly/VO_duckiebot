# Visual Odometry for Duckiebot

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

## Data collection (without ground projection)
  * Run camera_node on your duckiebot (1st terminal)
      ```
      $ docker -H razor.local run -it --net host --privileged --name base -v /data:/data duckietown/rpi-duckiebot-base:master18 /bin/bash
      $ roslaunch duckietown camera.launch veh:="razor" raw:="false"
      ```
  * Run Vicon on your desktop (2nd terminal)
      ```
      $ cd project_VO_ws && source devel/setup.bash
      $ roslaunch ros_vrpn_client mrasl_vicon_duckiebot.launch
      ```
  * Record data on your desktop (3rd terminal)
      ```
      $ export ROS_MASTER_URI=http://razor.local:11311/      
      $ rosbag record /razor/camera_node/image/compressed /duckiebot_razor/vrpn_client/estimated_odometry
      ```

## Decoder and Synchronization (on your desktop)
NOTE: the reason that we did not run decoder_node on Duckiebot because it run at very low frequency
  * Run roscore (1st terminal)
    ```
    $ roscore
    ```
  * Play your bag file (2nd terminal)
    ```
    $ rosplay <your_bag> --topic rosbag play razor_2.bag --topic /razor/camera_node/image/compressed /duckiebot_razor/vrpn_client/estimated_odometry
    ```
  * Run decoder_node at maximum 30Hz on your desktop (3rd terminal)
    ```
    roslaunch vo_duckiebot decoder_node.launch veh:="razor"
    ```
  * Check image_raw published at maximum 30Hz
    ```
    rostopic hz /razor/camera_node/image/raw
    ```
    Even we run this node at 30Hz, this topic is published at maximum 20Hz!
  * Run synchronization_node
  * Record new data

## Ground projection: to do
  * can not run ground_projection locally
  * run on duckiebot => segment is not published (00-infrastructure/duckietown_msgs/msg/Segment.msg)
  * may bedu

## Data export
  * txt file from
  * png image from image/raw
      ```

      ```
  * png image from Segment.msg  
## VISO2
  * Running offline
  * Running online

## Deep learning 1
## Deep learning n

## TO DO: presentation, new video after camera calibration 
