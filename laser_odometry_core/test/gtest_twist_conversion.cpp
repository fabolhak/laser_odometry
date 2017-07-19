#include <laser_odometry_core/conversion.h>

#include <gtest/gtest.h>

const double epsilon = 1e-8;

#define CHECK_QUATERNION_NEAR(_q1, _q2, _epsilon)        \
  EXPECT_NEAR(_q1.angle(_q2), 0, _epsilon);            \

#define CHECK_TRANSFORMS_NEAR(_out, _expected, _eps)                     \
  EXPECT_NEAR(_out.getOrigin().x(), _expected.getOrigin().x(), _eps);    \
  EXPECT_NEAR(_out.getOrigin().y(), _expected.getOrigin().y(), _eps);    \
  EXPECT_NEAR(_out.getOrigin().z(), _expected.getOrigin().z(), _eps);    \
  CHECK_QUATERNION_NEAR(_out.getRotation(),  _expected.getRotation(), _eps);

TEST(twist_conversion_tests, identity)
{
  tf::Transform delta_pose;

  delta_pose.setIdentity();

  geometry_msgs::Twist twist = laser_odometry::deltaPoseToTwist(delta_pose);

  ASSERT_EQ(twist.linear.x, 0);
  ASSERT_EQ(twist.linear.y, 0);
  ASSERT_EQ(twist.linear.z, 0);
  ASSERT_EQ(twist.angular.x, 0);
  ASSERT_EQ(twist.angular.y, 0);
  ASSERT_EQ(twist.angular.z, 0);

  tf::Transform delta_pose_back = laser_odometry::twistToDeltaPose(twist);

  CHECK_TRANSFORMS_NEAR(delta_pose_back, delta_pose, epsilon);
}

TEST(twist_conversion_tests, translation_only)
{
  tf::Transform delta_pose(tf::createQuaternionFromRPY(0, 0, 0),
                           tf::Vector3(0.5, 0.16, 0.79));

  geometry_msgs::Twist twist = laser_odometry::deltaPoseToTwist(delta_pose);

  tf::Transform delta_pose_back = laser_odometry::twistToDeltaPose(twist);

  CHECK_TRANSFORMS_NEAR(delta_pose_back, delta_pose, epsilon);
}

TEST(twist_conversion_tests, rotation_only)
{
  tf::Transform delta_pose(tf::createQuaternionFromRPY(0.1, 0.5, -0.3),
                           tf::Vector3(0, 0, 0));

  geometry_msgs::Twist twist = laser_odometry::deltaPoseToTwist(delta_pose);

  tf::Transform delta_pose_back = laser_odometry::twistToDeltaPose(twist);

  CHECK_TRANSFORMS_NEAR(delta_pose_back, delta_pose, epsilon);
}

TEST(twist_conversion_tests, rotation_translation)
{
  tf::Transform delta_pose(tf::createQuaternionFromRPY(0.05, -0.7, 0.444),
                           tf::Vector3(0.5, 0.16, 0.79));

  geometry_msgs::Twist twist = laser_odometry::deltaPoseToTwist(delta_pose);

//  geometry_msgs::Transform tf_msg;
//  tf::transformTFToMsg(delta_pose, tf_msg);
//  ROS_INFO_STREAM(tf_msg);

  tf::Transform delta_pose_back = laser_odometry::twistToDeltaPose(twist);

//  tf::transformTFToMsg(delta_pose_back, tf_msg);
//  ROS_INFO_STREAM(tf_msg);

  CHECK_TRANSFORMS_NEAR(delta_pose_back, delta_pose, epsilon);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
