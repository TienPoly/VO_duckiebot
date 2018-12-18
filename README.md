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

## Data collection
  * Run camera_node on your duckiebot (1st terminal)
<pre><code>$ docker -H <b>hostname</b>.local run -it --net host --privileged --name base -v /data:/data duckietown/rpi-duckiebot-base:master18 /bin/bash
$ roslaunch duckietown camera.launch veh:="razor" raw:="false"
</code></pre>


  * Run joystick container (2nd terminal)
      ```
      $ docker -H razor.local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18
      ```
  * Run Vicon on your desktop (same 2nd terminal)
      ```
      $ cd project_VO_ws && source devel/setup.bash
      $ roslaunch ros_vrpn_client mrasl_vicon_duckiebot.launch
      ```
  * Record data on your desktop (3rd terminal)
      ```
      $ export ROS_MASTER_URI=http://razor.local:11311/      
      $ rosbag record /razor/camera_node/image/compressed /duckiebot_razor/vrpn_client/estimated_odometry
      ```

  An example of this bag file: [razor_3.bag](https://drive.google.com/drive/folders/1I7cswHQ0SAr3dja1L5zuYut4Grgubu1t)

## Decoder and Synchronization (on your desktop)
NOTE: by default, decoder_node is run on Duckiebot at very low frequency (2Hz) due to limited computation. To get more images for deep learning, we run this node on a desktop.  
   * Run roscore (1st terminal)
     ```
     $ roscore
     ```
   * Play and Get camera info (2nd & 3rd terminals)
     ```
     $ rosbag play razor_3.bag
     $ rostopic echo /razor/camera_node/camera_info
     ```
   * Play and Run decoder_node at maximum 30Hz on your desktop (2nd & 3rd terminals)
     ```
     $ rosbag play razor_3.bag --topic /razor/camera_node/image/compressed  /duckiebot_razor/vrpn_client/estimated_odometry
     $ cd project_VO_ws && source devel/setup.bash
     $ roslaunch vo_duckiebot decoder_node.launch veh:="razor" param_file_name:="decoder_30Hz"
     ```
   * Check image_raw published at maximum 30Hz (4th terminal)
     ```
     rostopic hz /razor/camera_node/image/raw
     ```

   Even we run this node at 30Hz, this topic is published at maximum 20Hz!

  * Run synchronization_node (5th terminal): synchronization between image/raw and vicon data
    ```
    $ cd project_VO_ws && source devel/setup.bash
    $ roslaunch vo_duckiebot data_syn.launch
    ```
  * Record new data (4th terminal)
    ```
    $ rosbag record /razor/camera_node/image/raw /razor/vicon_republish/pose
    ```

    An example of the new bag file: [razor_3_syn.bag](https://drive.google.com/drive/folders/1I7cswHQ0SAr3dja1L5zuYut4Grgubu1t)

## Ground projection: to do
  * can not run ground_projection locally
  * run on duckiebot => segment is not published (00-infrastructure/duckietown_msgs/msg/Segment.msg)
  * to run at duckietown (A222)


## Data export
  * txt file from bag using MATLAB: run script_to_run.m with your new bag file
  * png image from image/raw
    ```
    $ ./bag2img.py razor_3_syn.bag images_30Hz/ /razor/camera_node/image/raw
    ```
  * png image from Segment.msg: TO DO

    An example of the text file and png images: [Duckiebot](https://drive.google.com/drive/folders/1I7cswHQ0SAr3dja1L5zuYut4Grgubu1t)


## VISO2: TO DO
  * Offline
  * Online

## DEEP LEARNING 1
## DEEP LEARNING n

## TO DO: presentation, new video with camera calibration, viso2, other direct method, Ground projection, deep learning
