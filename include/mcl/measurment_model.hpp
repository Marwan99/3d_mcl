#pragma once

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mcl/pose.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

// pcl::PointXYZI
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MeasurementModel
{
  ros::NodeHandle nh_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Publisher tf_scan_publisher_;

  PointCloudT::Ptr scan_cloud_;
  PointCloudT::Ptr map_cloud_;
  pcl::VoxelGrid<PointT> down_size_filter_;
  pcl::KdTreeFLANN<PointT>::Ptr kd_tree_;
  pcl::PassThrough<PointT> pass_filter_;

  octomap::AbstractOcTree* octomap_tree_;

  bool map_availble_;

public:
  bool scan_available;
  ros::Time scan_time;

  MeasurementModel(ros::NodeHandle& nh);

  void scan_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr);
  // void map_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr);
  void map_callback(const octomap_msgs::OctomapConstPtr& ocotmap_msg_ptr);
  void calculate_weights(std::vector<pose>& particles);

  double normal_dist(double x, double std_dev);
};
