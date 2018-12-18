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
      <pre><code>$ docker -H <i>hostname</i>.local run -it --net host --privileged --name base -v /data:/data duckietown/rpi-duckiebot-base:master18 /bin/bash
    $ roslaunch duckietown camera.launch veh:="<i>hostname</i>" raw:="false"
      </code></pre>
  * Check camera (2nd terminal)
      <pre><code>$ cd project_VO_ws && source devel/setup.bash
    $ export ROS_MASTER_URI=http://<i>hostname</i>.local:11311/
    $ rqt
      </code></pre>
  * Run joystick container (2nd terminal or using portainer)
      <pre><code>$ docker -H <i>hostname</i>.local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18
      </code></pre>
  * Run Vicon on your desktop (2nd terminal): the Vicon object name is '*duckiebot_hostname*'
      <pre><code>$ cd project_VO_ws && source devel/setup.bash
    $ export ROS_MASTER_URI=http://<i>hostname</i>.local:11311/
    $ roslaunch ros_vrpn_client mrasl_vicon_duckiebot.launch object_name:=<i>duckiebot_hostname</i>
      </code></pre>
  * Making your Duckiebot move and Record data on your desktop (3rd terminal)
      <pre><code>$ export ROS_MASTER_URI=http://<i>hostname</i>.local:11311/      
    $ rosbag record /<i>hostname</i>/camera_node/camera_info /<i>hostname</i>/camera_node/image/compressed <i>/duckiebot_hostname</i>/vrpn_client/estimated_odometry
      </code></pre>

  An example of this bag file: [razor_3.bag](https://drive.google.com/drive/folders/1I7cswHQ0SAr3dja1L5zuYut4Grgubu1t)

## Decoder and Synchronization (on your desktop)
NOTE: by default, decoder_node is run on Duckiebot at very low frequency (2Hz) due to limited computation. To get more images for deep learning, we run this node on a local desktop.  
   * Run roscore (1st terminal)

   * Run decoder_node at 10Hz (maximum 30Hz) on your desktop (2nd & 3rd terminals)
     <pre><code>$ rosbag play <i>bag_file</i>.bag --topic /<i>hostname</i>/camera_node/image/compressed  <i>/duckiebot_hostname</i>/vrpn_client/estimated_odometry
     $ cd project_VO_ws && source devel/setup.bash
     $ roslaunch vo_duckiebot decoder_node.launch veh:="<i>hostname</i>" param_file_name:="decoder_10Hz" </code></pre>

   * Run synchronization_node (3th terminal): synchronization between image/raw and vicon data
       <pre><code>$ cd project_VO_ws && source devel/setup.bash
     $ roslaunch vo_duckiebot data_syn.launch veh:="<i>hostname</i>" veh_vicon:="<i>duckiebot_hostname</i>" </code></pre>

   * Record new data (4th terminal)
       <pre><code>$ rosbag record /<i>hostname</i>/camera_node/image/raw /<i>hostname</i>/vicon_republish/pose </code></pre>

   * Verify camera info and Check image_raw published at 10Hz
       <pre><code>$ rostopic echo /<i>hostname</i>/camera_node/camera_info
      $ rostopic hz /<i>hostname</i>/camera_node/image/raw </code></pre>

     Even we run this node at 10Hz, this topic is published at about 8Hz!

    An example of the new bag file: [razor_3_syn.bag](https://drive.google.com/drive/folders/1I7cswHQ0SAr3dja1L5zuYut4Grgubu1t)

## Ground projection: to do
  * can not run ground_projection locally
  * run on duckiebot => segment is not published (00-infrastructure/duckietown_msgs/msg/Segment.msg)
  * to run at duckietown (A222)


## Data export
  * txt file from bag using MATLAB: run script_to_run.m with your new bag file
  * png image from image/raw: create a new folder, e.g. `images_10Hz`
    <pre><code>$ ./bag2img.py <i>bag_file_syn</i>.bag images_10Hz/ /<i>hostname</i>/camera_node/image/raw
    </code></pre>

    An example of the text file and png images: [Duckiebot](https://drive.google.com/drive/folders/1I7cswHQ0SAr3dja1L5zuYut4Grgubu1t)


## VISO2: TO DO
  * Offline
  * Online

## DEEP LEARNING 1
## DEEP LEARNING n

## TO DO: presentation, new video with camera calibration, viso2, other direct method, Ground projection, deep learning
