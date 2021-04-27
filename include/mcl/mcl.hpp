#pragma once

#include <vector>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include <mcl/pose.hpp>
#include <mcl/motion_model.hpp>
#include <mcl/measurment_model.hpp>

class MCL
{
  ros::NodeHandle nh_;
  ros::Publisher vis_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber init_pose_sub_;
  ros::ServiceServer save_traj_serv_;

  bool run_on_start_;
  std::vector<pose> init_particles_;
  double init_x_;
  double init_y_;
  double init_yaw_;

  bool tf_initialized_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaseter_;
  geometry_msgs::TransformStamped map_odom_tf_;

  std::vector<pose> particles;

  MotionModel motion_model;
  MeasurementModel measurement_model;
  
  // parameters
  bool pub_tf_;
  std::string trajectory_csv_;

  std::vector<nav_msgs::Odometry> trajectory_;

public:
  MCL(ros::NodeHandle& nh);

  void filter();
  void publish_markers();
  void low_var_respampling();
  bool normalise_weights();
  void publish_estimated_pose();
  void init_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg_ptr);
  bool save_trajectory_to_csv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};
