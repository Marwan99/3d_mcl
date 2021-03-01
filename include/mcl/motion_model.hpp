#pragma once

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>

#include <mcl/pose.hpp>


class MotionModel
{
  ros::NodeHandle nh_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber odom_subscriber_;

  geometry_msgs::Pose prev_odom_;
  geometry_msgs::Pose latest_odom_;

  gtsam::PreintegratedImuMeasurements *imu_integrator;
  gtsam::NavState prev_state_odom_;
  gtsam::imuBias::ConstantBias imu_bias;

  pose inc_pose_;

  double prev_time_;

public:
  bool odom_initialized;
  bool moved;

  MotionModel(ros::NodeHandle& nh);

  void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_raw);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg_ptr);
  void update_pose(std::vector<pose> &particle);
  void reset_pre_integration(const nav_msgs::Odometry & latest_estimate);

  double normalize(double value);
  double angle_diff(double a, double b);  
};
