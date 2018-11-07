#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "common.h"

#include <vo_duckiebot/DuckiebotData.h>

// VICON callback
vo_duckiebot::EigenOdometry gOdometry;
void OdometryCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  vo_duckiebot::eigenOdometryFromMsg(odom_msg, &gOdometry);                     // new measurement from Vicon
}

/*
Camera callback
Docker Ros-picam:       https://github.com/duckietown/rpi-duckiebot-ros-picam
ROS Pi Camera launch:   https://github.com/duckietown/Software/blob/master18/catkin_ws/src/00-infrastructure/duckietown/launch/camera.launch
ROS Pi Camera node:     https://github.com/duckietown/Software/tree/master18/catkin_ws/src/05-teleop/pi_camera
                        https://github.com/duckietown/Software/tree/master18/catkin_ws/src/00-infrastructure/duckietown/config/baseline/pi_camera

camera_node: publish /camera_node/image/compressed at 30Hz
decoder_node: subscribe /camera_node/image/compressed
              republish /decoder_node/image/compressed at 2Hz => remap to /camera_node/image/compressed => Conflict !
              publish /decoder_node/image/raw at 2Hz => remap to /camera_node/image/raw

Question 1: 30 Hz => too fast ?
Question 2: how to run ros-picam at 30Hz
    Sol 1: can run ros-picam decoder node at 30Hz => rosbag record
    Sol 2: run locally decoder_node
        python: ok
        C++: to do
Question 3: max 20Hz => off-line image processing ?
*/


int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collection_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("data_collection_node main started");

  ros::Subscriber odometry_sub_;
  odometry_sub_ = nh.subscribe(vo_duckiebot::kDefaultOdometryTopic, 1, OdometryCallback);

  ros::Publisher duckiebot_pub_;
  duckiebot_pub_ = nh.advertise<vo_duckiebot::DuckiebotData>(vo_duckiebot::default_topics::DUCKIEBOT_REPUBLISH, 1);

  Eigen::Vector3d velocity_W ;
  Eigen::Vector3d euler_angles;

  ros::Rate r(200);

  while(ros::ok()) {
      vo_duckiebot::DuckiebotDataPtr duckiebot_data_msg(new vo_duckiebot::DuckiebotData);

      // Note: Odometry => linear and angulare velocities in body frame
      // Eigen::Matrix3d R_W_B = gOdometry.orientation.toRotationMatrix();      // ZYX order
      // velocity_W =  R_W_B * gOdometry.velocity;
      // double psi = atan2(R_W_B(1,0),R_W_B(0,0));
      // double Psi = gOdometry.getYaw();                                       // same result
      // double phi  = atan2(R_W_B(2,1),R_W_B(2,2));
      // double theta = asin(-R_W_B(2,0));

      velocity_W = gOdometry.getVelocityWorld();
      gOdometry.getEulerAngles(&euler_angles);

      //ROS_INFO("x = %f, y = %f, z = %f",gOdometry.position.x(),gOdometry.position.y(),gOdometry.position.z());

      duckiebot_data_msg->header.stamp  =  ros::Time::now();
      duckiebot_data_msg->frame_id = "vicon";
      duckiebot_data_msg->vicon_position_W        = vo_duckiebot::geoVectorFromEig(gOdometry.position);
      duckiebot_data_msg->vicon_velocity_W        = vo_duckiebot::geoVectorFromEig(velocity_W);
      duckiebot_data_msg->vicon_velocity_B        = vo_duckiebot::geoVectorFromEig(gOdometry.velocity);
      duckiebot_data_msg->vicon_euler_angles      = vo_duckiebot::geoVectorFromEig(euler_angles);
      duckiebot_data_msg->vicon_rotation_speed_B  = vo_duckiebot::geoVectorFromEig(gOdometry.angular_velocity);

      duckiebot_pub_.publish(duckiebot_data_msg);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
