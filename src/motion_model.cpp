#include <mcl/motion_model.hpp>

#include <math.h>
#include <random>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

MotionModel::MotionModel(ros::NodeHandle& nh)
{
  nh_ = nh;

  imu_subscriber_ =
      nh_.subscribe<sensor_msgs::Imu>("/imu/data", 20000, &MotionModel::imu_callback, this);
  odom_subscriber_ =
      nh_.subscribe<nav_msgs::Odometry>("/encoder_odom", 1, &MotionModel::odom_callback, this);

  nh.param<double>("/mcl/translation_threshold", trans_thresh_, 0.1);
  nh.param<double>("/mcl/rotation_threshold", rot_thresh_, 0.35); // ~20 deg

  prev_time_ = ros::Time().now().toSec();

  boost::shared_ptr<gtsam::PreintegrationParams> p =
      gtsam::PreintegrationParams::MakeSharedU(9.80511);
  p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) *
                               pow(1.2154843674317246e-02, 2);  // acc white noise in continuous
  p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) *
                           pow(2.5532119453813736e-03, 2);  // gyro white noise in continuous
  p->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      pow(1e-4, 2);  // error committed in integrating position from velocities

  gtsam::imuBias::ConstantBias prior_imu_bias;
  imu_integrator = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
  imu_bias = gtsam::imuBias::ConstantBias();

  reset_pre_integration();

  odom_initialized = false;

  prev_odom_.position.x = 0;
  prev_odom_.position.y = 0;

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, 0);
  prev_odom_.orientation = tf2::toMsg(myQuaternion);
}

void MotionModel::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr)
{
  ROS_DEBUG("Odom message received.");

  if (!odom_initialized)
  {
    prev_odom_ = odom_msg_ptr->pose.pose;
    odom_initialized = true;
  }

  latest_odom_ = odom_msg_ptr->pose.pose;
}

void MotionModel::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_raw)
{
  ROS_DEBUG("IMU message received.");
  sensor_msgs::Imu imu_msg = *imu_raw;

  double dt = imu_msg.header.stamp.toSec() - prev_time_;
  dt = (dt <= 0) ? (1.0 / 100.0) : dt;
  prev_time_ = imu_msg.header.stamp.toSec();

  imu_integrator->integrateMeasurement(
      gtsam::Vector3(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                     imu_msg.linear_acceleration.z),
      gtsam::Vector3(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                     imu_msg.angular_velocity.z),
      dt);

  gtsam::NavState current_state = imu_integrator->predict(prev_state_odom_, imu_bias);

  gtsam::Pose3 imu_pose = gtsam::Pose3(current_state.quaternion(), current_state.position());

  inc_pose_.x = imu_pose.translation().x();
  inc_pose_.y = imu_pose.translation().y();
  inc_pose_.yaw = imu_pose.rotation().rpy().z();
}

void MotionModel::update_pose(std::vector<pose>& particles)
{
  ROS_DEBUG("Updating pose");

  double a_1 = 0.8;
  double a_2 = 0.8;
  double a_3 = 0.2;
  double a_4 = 0.2;

  double latest_yaw = tf2::getYaw(latest_odom_.orientation);
  double prev_yaw = tf2::getYaw(prev_odom_.orientation);

  double delta_x = latest_odom_.position.x - prev_odom_.position.x;
  double delta_y = latest_odom_.position.y - prev_odom_.position.y;

  double delta_rot_1 = angle_diff(std::atan2(delta_y, delta_x), prev_yaw);
  double delta_trans = std::hypot(delta_x, delta_y);
  double delta_rot_2 = angle_diff(angle_diff(latest_yaw, prev_yaw), delta_rot_1);

  if (delta_trans < trans_thresh_ && normalize(delta_rot_1 + delta_rot_2) < rot_thresh_)
  {
    moved = false;
    return;
  }
  else
    moved = true;

  double delta_rot_1_std = std::sqrt(a_1 * pow(delta_rot_1, 2) + a_2 * pow(delta_trans, 2));
  double delta_trans_std =
      std::sqrt(a_3 * pow(delta_trans, 2) + a_4 * pow(delta_rot_1, 2) + a_4 * pow(delta_rot_2, 2));
  double delta_rot_2_std = std::sqrt(a_1 * pow(delta_rot_2, 2) + a_2 * pow(delta_trans, 2));

  std::random_device device;
  std::mt19937 generator(device());

  std::normal_distribution<double> delta_rot_1_dist(0.0, delta_rot_1_std);
  std::normal_distribution<double> delta_trans_dist(0.0, delta_trans_std);
  std::normal_distribution<double> delta_rot_2_dist(0.0, delta_rot_2_std);

  for (auto& particle : particles)
  {
    double delta_rot_1_sampled = angle_diff(delta_rot_1, delta_rot_1_dist(generator));
    double delta_trans_sampled = delta_trans - delta_trans_dist(generator);
    double delta_rot_2_sampled = angle_diff(delta_rot_2, delta_rot_2_dist(generator));

    // ROS_ERROR_STREAM(delta_rot_1_sampled << " " << delta_rot_2_sampled);

    if (std::isnan(delta_rot_1_sampled))
      ROS_ERROR("isnan(delta_rot_1_sampled)");

    if (std::isnan(delta_trans_sampled))
      ROS_ERROR("isnan(delta_trans_sampled)");

    if (std::isnan(delta_rot_2_sampled))
      ROS_ERROR("isnan(delta_rot_2_sampled)");

    particle.x += delta_trans_sampled * std::cos(particle.yaw + delta_rot_1_sampled);
    particle.y += delta_trans_sampled * std::sin(particle.yaw + delta_rot_1_sampled);
    particle.yaw += delta_rot_1_sampled + delta_rot_2_sampled;
    // particle.yaw = normalize(particle.yaw);
  }

  prev_odom_ = latest_odom_;
}

void MotionModel::reset_pre_integration()
{
  imu_integrator->resetIntegrationAndSetBias(imu_bias);
  prev_state_odom_ = gtsam::NavState(gtsam::Pose3().identity(), gtsam::Velocity3(0, 0, 0));

  inc_pose_.x = 0;
  inc_pose_.y = 0;
}

double MotionModel::normalize(double angle)
{
  return (angle - (std::floor((angle + M_PI) / 2 * M_PI) * 2 * M_PI));
  // return angle - 2 * M_PI * std::floor((angle + M_PI) / (2 * M_PI));
}

double MotionModel::angle_diff(double a, double b)
{
  double angle = a - b;
  angle = fmod(angle, 2.0 * M_PI);

  if (angle <= M_PI && angle >= -M_PI)
    return angle;

  else if (angle > M_PI)
    return angle - 2.0 * M_PI;

  else
    return angle + 2.0 * M_PI;
}

// double MotionModel::angle_diff(double a, double b)
// {
//   a = normalize(a);
//   b = normalize(b);
//   auto d1 = a - b;
//   auto d2 = 2 * M_PI - std::abs(d1);

//   if (0 < d1) {
//     d2 = -d2;
//   }
//   // Determine if d1 or d2 should be used based on magnitude
//   if (std::abs(d1) < std::abs(d2)) {
//     return d1;
//   } else {
//     return d2;
//   }
// }
