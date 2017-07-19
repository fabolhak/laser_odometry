#ifndef _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_
#define _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_

#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

#include <tf_conversions/tf_eigen.h>

#include <sophus/se3.hpp>
#include <sophus/se2.hpp>

#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovariance.h>

namespace laser_odometry
{

namespace
{
typedef Sophus::SE3d SE3;
}

inline void deltaPoseToTwist(const tf::Transform& delta_pose,
                             const Eigen::Matrix<double, 6, 6>& delta_covariance,
                             geometry_msgs::Twist& twist_msg,
                             Eigen::Matrix<double, 6, 6>& twist_covariance)
{
  Eigen::Quaternion<double> quat;
  tf::quaternionTFToEigen(delta_pose.getRotation(), quat);

  Eigen::Vector3d tran;
  tf::vectorTFToEigen(delta_pose.getOrigin(), tran);

  const SE3 se3(quat, tran);

  const SE3::Tangent tangent_se3 = se3.log();

  twist_msg.linear.x = tangent_se3(0);
  twist_msg.linear.y = tangent_se3(1);
  twist_msg.linear.z = tangent_se3(2);

  twist_msg.angular.x = tangent_se3(3);
  twist_msg.angular.y = tangent_se3(4);
  twist_msg.angular.z = tangent_se3(5);

//  Eigen::Matrix<double, 7, 6> jac = se3.internalJacobian();

//  /*twist_covariance =*/ jac.transpose() * delta_covariance /** jac*/;
}

inline geometry_msgs::Twist deltaPoseToTwist(const tf::Transform& delta_pose)
{
  Eigen::Quaternion<double> quat;
  tf::quaternionTFToEigen(delta_pose.getRotation(), quat);

  Eigen::Vector3d tran;
  tf::vectorTFToEigen(delta_pose.getOrigin(), tran);

  const SE3 se3(quat, tran);

  const SE3::Tangent tangent_se3 = se3.log();

  geometry_msgs::Twist twist_msg;

  twist_msg.linear.x = tangent_se3(0);
  twist_msg.linear.y = tangent_se3(1);
  twist_msg.linear.z = tangent_se3(2);

  twist_msg.angular.x = tangent_se3(3);
  twist_msg.angular.y = tangent_se3(4);
  twist_msg.angular.z = tangent_se3(5);

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

  tf::Quaternion quat;
  tf::quaternionEigenToTF(se3.unit_quaternion(), quat);

  tf::Vector3 tran;
  tf::vectorEigenToTF(se3.translation(), tran);

  return tf::Transform(quat, tran);
}

inline void toRos(const Eigen::Matrix<double, 6, 6>& covariance,
                  geometry_msgs::PoseWithCovariance::_covariance_type& covariance_msg)
{
  covariance_msg.at(0)  = covariance(0,0);
  covariance_msg.at(1)  = covariance(0,1);
  covariance_msg.at(2)  = covariance(0,2);
  covariance_msg.at(3)  = covariance(0,3);
  covariance_msg.at(4)  = covariance(0,4);
  covariance_msg.at(5)  = covariance(0,5);

  covariance_msg.at(6)  = covariance(1,0);
  covariance_msg.at(7)  = covariance(1,1);
  covariance_msg.at(8)  = covariance(1,2);
  covariance_msg.at(9)  = covariance(1,3);
  covariance_msg.at(10) = covariance(1,4);
  covariance_msg.at(11) = covariance(1,5);

  covariance_msg.at(12) = covariance(2,0);
  covariance_msg.at(13) = covariance(2,1);
  covariance_msg.at(14) = covariance(2,2);
  covariance_msg.at(15) = covariance(2,3);
  covariance_msg.at(16) = covariance(2,4);
  covariance_msg.at(17) = covariance(2,5);

  covariance_msg.at(18) = covariance(3,0);
  covariance_msg.at(19) = covariance(3,1);
  covariance_msg.at(20) = covariance(3,2);
  covariance_msg.at(21) = covariance(3,3);
  covariance_msg.at(22) = covariance(3,4);
  covariance_msg.at(23) = covariance(3,5);

  covariance_msg.at(24) = covariance(4,0);
  covariance_msg.at(25) = covariance(4,1);
  covariance_msg.at(26) = covariance(4,2);
  covariance_msg.at(27) = covariance(4,3);
  covariance_msg.at(28) = covariance(4,4);
  covariance_msg.at(29) = covariance(4,5);

  covariance_msg.at(30) = covariance(5,0);
  covariance_msg.at(31) = covariance(5,1);
  covariance_msg.at(32) = covariance(5,2);
  covariance_msg.at(33) = covariance(5,3);
  covariance_msg.at(34) = covariance(5,4);
  covariance_msg.at(35) = covariance(5,5);
}

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CORE_LASER_ODOMETRY_CONVERSION_H_ */
