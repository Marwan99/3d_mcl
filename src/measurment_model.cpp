#include <math.h>
#include <random>
#include <mcl/measurment_model.hpp>

MeasurementModel::MeasurementModel(ros::NodeHandle& nh)
{
  nh_ = nh;
  scan_subscriber_ =
      nh_.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &MeasurementModel::scan_callback, this);
  map_subscriber_ =
      nh_.subscribe<sensor_msgs::PointCloud2>("/lio_sam/mapping/map_global", 1, &MeasurementModel::map_callback, this);

  tf_scan_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/tf_scan", 1);

  scan_available = false;

  scan_cloud_.reset(new pcl::PointCloud<PointT>());
  map_cloud_.reset(new pcl::PointCloud<PointT>());
  kd_tree_.reset(new pcl::KdTreeFLANN<PointT>());

  down_size_filter_.setLeafSize(3, 3, 3);

  pass_filter_.setFilterFieldName ("z");
  pass_filter_.setFilterLimits (0.5, 5.0);
}

void MeasurementModel::scan_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr)
{
  ROS_DEBUG("scan received");
  pcl::fromROSMsg(*cloud_msg_ptr, *scan_cloud_);

  pass_filter_.setInputCloud(scan_cloud_);
  pass_filter_.filter(*scan_cloud_);

  scan_time = cloud_msg_ptr->header.stamp;

  // down_size_filter_.setInputCloud(scan_cloud_);
  // down_size_filter_.filter(*scan_cloud_);

  scan_available = true;
}

void MeasurementModel::map_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr)
{
  ROS_DEBUG("map received");
  pcl::fromROSMsg(*cloud_msg_ptr, *map_cloud_);

  pass_filter_.setInputCloud(map_cloud_);
  pass_filter_.filter(*map_cloud_);

  kd_tree_->setInputCloud(map_cloud_);

  map_subscriber_.shutdown();
}

void MeasurementModel::calculate_weights(std::vector<pose>& particles)
{
  ROS_DEBUG("Updating particle weights");

  std::default_random_engine generator;
  std::uniform_real_distribution<double> uni_dist(0.0, 0.1);

  for (auto & particle : particles)
  {
    PointCloudT::Ptr temp_cloud(new PointCloudT(*scan_cloud_));
    std::vector<int> closest_point_ind;
    std::vector<float> closest_point_sqr_dist;

    // Tranform scan to global frame
    Eigen::Affine3f tf_affine = pcl::getTransformation(particle.x, particle.y, 0, 0, 0, particle.yaw);
    pcl::transformPointCloud(*temp_cloud, *temp_cloud, tf_affine.cast<double>().matrix());

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*temp_cloud, cloud_msg);
    cloud_msg.header.frame_id = "odom";
    cloud_msg.header.stamp = ros::Time::now();
    tf_scan_publisher_.publish(cloud_msg);

    // particle.weight = 1.0;
    double q = 1.0;
    for (int i = 0; i < temp_cloud->size(); i+=500)
    {
      PointT cur_point = temp_cloud->points[i];

      int k_found = kd_tree_->nearestKSearch(cur_point, 1, closest_point_ind, closest_point_sqr_dist);
      if (k_found > 0)
      {
        if(sqrt(closest_point_sqr_dist[0]) > 5.0)
        {
          q = 0;
          continue;  
        }
      }

      double prob_dist = normal_dist((double)sqrt(closest_point_sqr_dist[0]), 1.5);
      q *= 0.9*prob_dist+ 0.05*uni_dist(generator) / 100.0;
    }
    particle.weight = q;
  }
}

double MeasurementModel::normal_dist(double x, double std_dev)
{
  return exp(-0.5 * pow(x / std_dev, 2)) / (std_dev * 2.506628);
}
