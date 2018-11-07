#ifndef VO_DUCKIEBOT_COMMON_H
#define VO_DUCKIEBOT_COMMON_H

#include <assert.h>
#include <nav_msgs/Odometry.h>

namespace vo_duckiebot {
// Default values.
static const std::string kDefaultNamespace = "";
static const std::string kDefaultOdometryTopic ="odometry";

// Default physics parameters.
static constexpr double kDefaultGravity = 9.81;
static constexpr double kDefaultPi      = 3.14159;

namespace default_topics {
  static constexpr char VICON_REPUBLISH[] = "vicon_republish/pose";
}


inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
  {
    assert(euler_angles != NULL);

    *euler_angles << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
        atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
}
inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
  *skew_matrix << 0, -vector.z(), vector.y(),
                  vector.z(), 0, -vector.x(),
                  -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
  *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}

inline geometry_msgs::Vector3 geoVectorFromEig(Eigen::Vector3d& vectorEig) {
  geometry_msgs::Vector3 vectorGeo;
  vectorGeo.x = vectorEig[0];
  vectorGeo.y = vectorEig[1];
  vectorGeo.z = vectorEig[2];
  return vectorGeo;
}

inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quaternionFromMsg(
    const geometry_msgs::Quaternion& msg) {
  // Make sure this always returns a valid Quaternion, even if the message was
  // uninitialized or only approximately set.
  Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
  if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
    quaternion.setIdentity();
  } else {
    quaternion.normalize();
  }
  return quaternion;
}
inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;       // orientation_W_B
  Eigen::Vector3d velocity;             // [u v w] velocity expressed in the Body frame!
  Eigen::Vector3d angular_velocity;     // [p q r] angular velocity in the Body frame too!

  inline double getYaw() const { return yawFromQuaternion(orientation); }
  inline void getEulerAngles(Eigen::Vector3d* euler_angles) const {
    getEulerAnglesFromQuaternion(orientation, euler_angles);
  }
  inline Eigen::Vector3d getVelocityWorld() const {
    return orientation * velocity;    // orientation_W_B * velocity_B
  }
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position = vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = vector3FromMsg(msg->twist.twist.angular);
}

template<typename T> inline void GetRosParameter(const ros::NodeHandle& nh,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}
}
#endif // VO_DUCKIEBOT_COMMON_H
