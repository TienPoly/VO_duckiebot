#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "common.h"
#include <vo_duckiebot/Vicon.h>

bool gPublish;
vo_duckiebot::EigenOdometry gOdometry;

// VICON callback
void OdometryCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  vo_duckiebot::eigenOdometryFromMsg(odom_msg, &gOdometry);                     // new measurement from Vicon
}

/*
Camera callback
RESUME
		Docker:    https://github.com/duckietown/rpi-duckiebot-ros-picam
		Launch:    https://github.com/duckietown/Software/blob/master18/catkin_ws/src/00-infrastructure/duckietown/launch/camera.launch
		Nodes:     https://github.com/duckietown/Software/tree/master18/catkin_ws/src/05-teleop/pi_camera
		Yaml:      https://github.com/duckietown/Software/tree/master18/catkin_ws/src/00-infrastructure/duckietown/config/baseline/pi_camera

		Topics
				/camera_node/image/compressed       published by camera_node, at 30Hz
				/decoder_node/image/compressed      published by decoder_node, at 2Hz
				/camera_node/image/raw              published by decoder_node, at 2Hz (remap from /decoder_node/image/raw)

*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!gPublish){
    gPublish = true;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collection_node");
  ros::NodeHandle nh;
  //ros::NodeHandle pnh("~");
  ROS_INFO("data_collection_node main started");

  ros::Subscriber odometry_sub_;
  odometry_sub_ = nh.subscribe(vo_duckiebot::kDefaultOdometryTopic, 1, OdometryCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::Publisher duckiebot_pub_;
  duckiebot_pub_ = nh.advertise<vo_duckiebot::Vicon>(vo_duckiebot::default_topics::VICON_REPUBLISH, 1);

  Eigen::Vector3d velocity_W ;
  Eigen::Vector3d euler_angles;

  ros::Rate r(60);
  gPublish = false;

  while(ros::ok()) {
      if (gPublish){
          vo_duckiebot::ViconPtr vicon_data_msg(new vo_duckiebot::Vicon);

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

          vicon_data_msg->header.stamp  =  ros::Time::now();
          vicon_data_msg->header.frame_id = "vicon";
          vicon_data_msg->vicon_position_W        = vo_duckiebot::geoVectorFromEig(gOdometry.position);
          vicon_data_msg->vicon_velocity_W        = vo_duckiebot::geoVectorFromEig(velocity_W);
          vicon_data_msg->vicon_velocity_B        = vo_duckiebot::geoVectorFromEig(gOdometry.velocity);
          vicon_data_msg->vicon_euler_angles      = vo_duckiebot::geoVectorFromEig(euler_angles);
          vicon_data_msg->vicon_rotation_speed_B  = vo_duckiebot::geoVectorFromEig(gOdometry.angular_velocity);

          duckiebot_pub_.publish(vicon_data_msg);
          gPublish = false;
        }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
