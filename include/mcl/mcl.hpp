#pragma once

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <mcl/pose.hpp>
#include <mcl/motion_model.hpp>
#include <mcl/measurment_model.hpp>

class MCL
{
  ros::NodeHandle nh_;
  ros::Publisher vis_pub_;
  ros::Publisher pose_pub_;

  std::vector<pose> particles;

  MotionModel motion_model;
  MeasurementModel measurement_model;

  nav_msgs::Odometry cur_estimate;

public:
  MCL(ros::NodeHandle &nh);

  void filter();
  void publish_markers();
  void publish_estimated_pose();
  void low_var_respampling();
  void normalise_weights();
};
