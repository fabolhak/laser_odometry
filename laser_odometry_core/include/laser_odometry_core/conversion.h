#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_

#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

#include <tf_conversions/tf_eigen.h>

#include <sophus/se3.hpp>

namespace laser_odometry
{

typedef Sophus::SE3d SE3;

inline geometry_msgs::Twist deltaPoseToTwist(const tf::Transform& delta_pose)
{
  Eigen::Quaternion<double> quat;
  tf::quaternionTFToEigen(delta_pose.getRotation(), quat);

  Eigen::Vector3d tran;
  tf::vectorTFToEigen(delta_pose.getOrigin(), tran);

  const SE3 se3(quat, tran);

  const SE3::Tangent tangent = se3.log();

  ROS_INFO_STREAM("\n" << se3.matrix());
  ROS_INFO_STREAM("tangent " << tangent.transpose());

  geometry_msgs::Twist twist_msg;

  twist_msg.linear.x = tangent(0);
  twist_msg.linear.y = tangent(1);
  twist_msg.linear.z = tangent(2);

  twist_msg.angular.x = tangent(3);
  twist_msg.angular.y = tangent(4);
  twist_msg.angular.z = tangent(5);

  return twist_msg;
}

inline tf::Transform twistToDeltaPose(const geometry_msgs::Twist& twist_msg)
{
  SE3::Tangent tangent;
  tangent << twist_msg.linear.x,
             twist_msg.linear.y,
             twist_msg.linear.z,
             twist_msg.angular.x,
             twist_msg.angular.y,
             twist_msg.angular.z;

  const SE3 se3 = SE3::exp(tangent);

  ROS_INFO_STREAM("tangent " << tangent.transpose());
  ROS_INFO_STREAM("\n" << se3.matrix());

  tf::Quaternion quat;
  tf::quaternionEigenToTF(se3.unit_quaternion(), quat);

  tf::Vector3 tran;
  tf::vectorEigenToTF(se3.translation(), tran);

  return tf::Transform(quat, tran);
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_ */
