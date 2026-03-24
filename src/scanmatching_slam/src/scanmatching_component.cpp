// Copyright 2024 scanmatching_slam Authors
// Adapted from lidarslam_ros2 scanmatching_component
#include "scanmatching_slam/scanmatching_component.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <pcl/common/transforms.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>

namespace scanmatching_slam
{

ScanMatchingComponent::ScanMatchingComponent(const rclcpp::NodeOptions & options)
: Node("scanmatching_slam", options),
  tfbuffer_(this->get_clock()),
  listener_(nullptr)
{
  // Declare parameters
  registration_method_ = this->declare_parameter<std::string>("registration_method", "NDT");
  ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 5.0);
  ndt_num_threads_ = this->declare_parameter<int>("ndt_num_threads", 0);
  gicp_corr_dist_ = this->declare_parameter<double>("gicp_corr_dist", 1.0);
  trans_for_mapupdate_ = this->declare_parameter<double>("trans_for_mapupdate", 0.3);
  vg_size_for_input_ = this->declare_parameter<double>("vg_size_for_input", 0.05);
  vg_size_for_map_ = this->declare_parameter<double>("vg_size_for_map", 0.05);
  vg_size_for_matching_ = this->declare_parameter<double>("vg_size_for_matching", 0.15);
  scan_min_range_ = this->declare_parameter<double>("scan_min_range", 0.5);
  scan_max_range_ = this->declare_parameter<double>("scan_max_range", 4.0);
  num_targeted_cloud_ = this->declare_parameter<int>("num_targeted_cloud", 20);
  map_publish_period_ = this->declare_parameter<double>("map_publish_period", 5.0);
  use_odom_ = this->declare_parameter<bool>("use_odom", true);
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
  global_frame_id_ = this->declare_parameter<std::string>("global_frame_id", "map");
  robot_frame_id_ = this->declare_parameter<std::string>("robot_frame_id", "base_link");
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "/full_pointcloud");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");

  // Setup registration
  if (registration_method_ == "NDT") {
    auto ndt = std::make_shared<pclomp::NormalDistributionsTransform<PointT, PointT>>();
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(0.001);
    ndt->setMaximumIterations(35);
    if (ndt_num_threads_ > 0) {
      ndt->setNumThreads(ndt_num_threads_);
    }
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    registration_ = ndt;
    RCLCPP_INFO(get_logger(), "Using NDT registration (resolution=%.2f)", ndt_resolution_);
  } else if (registration_method_ == "GICP") {
    auto gicp = std::make_shared<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>>();
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_);
    gicp->setTransformationEpsilon(1e-6);
    gicp->setMaximumIterations(64);
    if (ndt_num_threads_ > 0) {
      gicp->setNumThreads(ndt_num_threads_);
    }
    registration_ = gicp;
    RCLCPP_INFO(get_logger(), "Using GICP registration (corr_dist=%.2f)", gicp_corr_dist_);
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown registration method: %s", registration_method_.c_str());
    return;
  }

  // TF
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  listener_ = std::make_shared<tf2_ros::TransformListener>(tfbuffer_);

  // Callback groups: separate heavy processing from lightweight timers
  processing_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Publishers
  map_array_pub_ = this->create_publisher<slam_msgs::msg::MapArray>("map_array", 1);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 1);
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);

  // Subscribers (heavy processing group)
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = processing_cb_group_;

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ScanMatchingComponent::cloudCallback, this, std::placeholders::_1),
    sub_opts);

  if (use_odom_) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanMatchingComponent::odomCallback, this, std::placeholders::_1));
  }

  modified_map_sub_ = this->create_subscription<slam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(1).reliable(),
    std::bind(&ScanMatchingComponent::mapArrayCallback, this, std::placeholders::_1));

  // Timers (separate group - can fire while NDT is processing)
  map_publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(map_publish_period_),
    std::bind(&ScanMatchingComponent::mapPublishTimerCallback, this),
    timer_cb_group_);

  tf_republish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ScanMatchingComponent::republishTF, this),
    timer_cb_group_);

  path_.header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "ScanMatchingComponent initialized");
  RCLCPP_INFO(get_logger(), "  Subscribing to cloud: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  Subscribing to odom: %s (use_odom=%s)",
    odom_topic_.c_str(), use_odom_ ? "true" : "false");
}

void ScanMatchingComponent::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = msg;
}

void ScanMatchingComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert to PCL
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);

  // Remove NaN and filter by range
  PointCloudT::Ptr filtered(new PointCloudT);
  filtered->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (dist >= scan_min_range_ && dist <= scan_max_range_) {
      filtered->points.push_back(pt);
    }
  }
  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;

  if (filtered->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Empty cloud after filtering");
    return;
  }

  // Fine cloud for map storage
  PointCloudT::Ptr cloud_fine(new PointCloudT);
  applyVoxelFilter(filtered, cloud_fine, vg_size_for_input_);

  // Coarse cloud for fast scan matching
  PointCloudT::Ptr cloud_coarse(new PointCloudT);
  applyVoxelFilter(filtered, cloud_coarse, vg_size_for_matching_);

  rclcpp::Time stamp = msg->header.stamp;

  if (!initial_cloud_received_) {
    initializeMap(cloud_fine, cloud_coarse, stamp);
  } else {
    receiveCloud(cloud_fine, cloud_coarse, stamp);
  }
}

void ScanMatchingComponent::initializeMap(
  PointCloudT::Ptr cloud_fine, PointCloudT::Ptr cloud_coarse, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(slam_mutex_);
  auto & cloud = cloud_fine;  // Store fine cloud in map
  RCLCPP_INFO(get_logger(), "Initializing map with first cloud (%zu fine, %zu coarse points)",
    cloud_fine->size(), cloud_coarse->size());

  current_pose_ = Eigen::Matrix4f::Identity();
  previous_pose_ = Eigen::Matrix4f::Identity();

  // Store first submap (cloud in LOCAL coordinates)
  SubMapData submap;
  submap.cloud = cloud;
  submap.pose = current_pose_;
  submap.distance = 0.0;
  submaps_.push_back(submap);

  // Save initial odom for reference
  if (use_odom_) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    prev_odom_ = latest_odom_;
  }

  initial_cloud_received_ = true;

  publishMapAndPose(current_pose_, stamp);

  // Publish initial map array
  slam_msgs::msg::MapArray map_array_msg;
  map_array_msg.header.stamp = stamp;
  map_array_msg.header.frame_id = global_frame_id_;
  map_array_msg.cloud_coordinate = slam_msgs::msg::MapArray::LOCAL;

  slam_msgs::msg::SubMap submap_msg;
  submap_msg.header.stamp = stamp;
  submap_msg.header.frame_id = global_frame_id_;
  submap_msg.distance = 0.0;

  // Pose from matrix
  Eigen::Affine3d affine(current_pose_.cast<double>());
  submap_msg.pose = tf2::toMsg(affine);

  // Cloud in LOCAL frame
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = robot_frame_id_;
  submap_msg.cloud = cloud_msg;

  map_array_msg.submaps.push_back(submap_msg);
  map_array_pub_->publish(map_array_msg);

  RCLCPP_INFO(get_logger(), "Map initialized with first submap");
}

void ScanMatchingComponent::receiveCloud(
  PointCloudT::Ptr cloud_fine, PointCloudT::Ptr cloud_coarse, const rclcpp::Time & stamp)
{
  // Phase 1: Build target cloud (needs slam_mutex_)
  PointCloudT::Ptr target_cloud;
  Eigen::Matrix4f initial_guess;
  {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (submaps_.empty()) {
      RCLCPP_WARN(get_logger(), "No submaps available for matching");
      return;
    }
    target_cloud = getTargetCloud();
    initial_guess = use_odom_ ? getOdomInitialGuess() : current_pose_;
  }

  if (target_cloud->empty()) {
    RCLCPP_WARN(get_logger(), "Target cloud is empty");
    return;
  }

  // Phase 2: NDT alignment (NO lock - this is the slow part)
  PointCloudT::Ptr target_downsampled(new PointCloudT);
  applyVoxelFilter(target_cloud, target_downsampled, vg_size_for_matching_);

  registration_->setInputTarget(target_downsampled);
  registration_->setInputSource(cloud_coarse);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Aligning cloud (%zu coarse pts) against target (%zu pts)",
    cloud_coarse->size(), target_downsampled->size());
  PointCloudT::Ptr result(new PointCloudT);
  registration_->align(*result, initial_guess);

  if (!registration_->hasConverged()) {
    RCLCPP_WARN(get_logger(), "Scan matching did not converge (score=%.3f)",
      registration_->getFitnessScore());
    return;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Scan matched (score=%.3f)", registration_->getFitnessScore());

  Eigen::Matrix4f matched_pose = registration_->getFinalTransformation();

  // Phase 3: Update state (needs slam_mutex_)
  {
    std::lock_guard<std::mutex> lock(slam_mutex_);

    Eigen::Vector3f delta_translation =
      matched_pose.block<3, 1>(0, 3) - previous_pose_.block<3, 1>(0, 3);
    double delta_dist = delta_translation.norm();

    total_distance_ += delta_dist;
    submap_distance_ += delta_dist;
    current_pose_ = matched_pose;

    publishMapAndPose(current_pose_, stamp);

    if (submap_distance_ >= trans_for_mapupdate_) {
      updateMap(cloud_fine, current_pose_);
      submap_distance_ = 0.0;
    }

    previous_pose_ = current_pose_;
  }

  // Update odom reference
  if (use_odom_) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    prev_odom_ = latest_odom_;
  }
}

Eigen::Matrix4f ScanMatchingComponent::getOdomInitialGuess()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);

  if (!latest_odom_ || !prev_odom_) {
    return current_pose_;
  }

  // Calculate relative motion from odom
  Eigen::Affine3d prev_odom_affine;
  Eigen::Affine3d curr_odom_affine;

  tf2::fromMsg(prev_odom_->pose.pose, prev_odom_affine);
  tf2::fromMsg(latest_odom_->pose.pose, curr_odom_affine);

  // Relative transform in odom frame
  Eigen::Matrix4f odom_delta =
    (prev_odom_affine.inverse() * curr_odom_affine).matrix().cast<float>();

  // Apply delta to current SLAM pose
  return current_pose_ * odom_delta;
}

PointCloudT::Ptr ScanMatchingComponent::getTargetCloud() const
{
  PointCloudT::Ptr target(new PointCloudT);

  // Use num_targeted_cloud most recent submaps
  int start_idx = std::max(0, static_cast<int>(submaps_.size()) - num_targeted_cloud_);

  for (int i = start_idx; i < static_cast<int>(submaps_.size()); ++i) {
    // Transform submap cloud from LOCAL to GLOBAL using submap pose
    PointCloudT::Ptr transformed(new PointCloudT);
    pcl::transformPointCloud(*(submaps_[i].cloud), *transformed, submaps_[i].pose);
    *target += *transformed;
  }

  return target;
}

void ScanMatchingComponent::updateMap(PointCloudT::Ptr cloud, const Eigen::Matrix4f & pose)
{
  // Store cloud in LOCAL coordinates (not transformed)
  SubMapData submap;
  submap.cloud = cloud;
  submap.pose = pose;
  submap.distance = total_distance_;
  submaps_.push_back(submap);

  RCLCPP_INFO(get_logger(), "New submap added (#%zu, dist=%.2f, total_dist=%.2f)",
    submaps_.size(), submap.distance, total_distance_);

  // Publish map array every time a new submap is added
  rclcpp::Time now = this->now();
  slam_msgs::msg::MapArray map_array_msg;
  map_array_msg.header.stamp = now;
  map_array_msg.header.frame_id = global_frame_id_;
  map_array_msg.cloud_coordinate = slam_msgs::msg::MapArray::LOCAL;

  for (size_t i = 0; i < submaps_.size(); ++i) {
    slam_msgs::msg::SubMap submap_msg;
    submap_msg.header.stamp = now;
    submap_msg.header.frame_id = global_frame_id_;
    submap_msg.distance = submaps_[i].distance;

    // Pose
    Eigen::Affine3d affine(submaps_[i].pose.cast<double>());
    submap_msg.pose = tf2::toMsg(affine);

    // Cloud in LOCAL frame
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*(submaps_[i].cloud), cloud_msg);
    cloud_msg.header.stamp = now;
    cloud_msg.header.frame_id = robot_frame_id_;
    submap_msg.cloud = cloud_msg;

    map_array_msg.submaps.push_back(submap_msg);
  }

  map_array_pub_->publish(map_array_msg);
}

void ScanMatchingComponent::publishMapAndPose(
  const Eigen::Matrix4f & pose, const rclcpp::Time & stamp)
{
  // Publish PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = global_frame_id_;
  Eigen::Affine3d affine(pose.cast<double>());
  pose_msg.pose = tf2::toMsg(affine);
  pose_pub_->publish(pose_msg);

  // Add to path
  path_.header.stamp = stamp;
  path_.poses.push_back(pose_msg);
  path_pub_->publish(path_);

  // Publish TF
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;

    if (use_odom_) {
      // Publish map -> odom
      // map_to_base = pose
      // odom_to_base is from TF or odom msg
      // map_to_odom = map_to_base * base_to_odom = pose * odom_to_base^{-1}
      transform.header.frame_id = global_frame_id_;
      transform.child_frame_id = odom_frame_id_;

      Eigen::Matrix4f odom_to_base = Eigen::Matrix4f::Identity();
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (latest_odom_) {
          Eigen::Affine3d odom_affine;
          tf2::fromMsg(latest_odom_->pose.pose, odom_affine);
          odom_to_base = odom_affine.matrix().cast<float>();
        }
      }

      Eigen::Matrix4f map_to_odom = pose * odom_to_base.inverse();
      Eigen::Isometry3d map_to_odom_iso(map_to_odom.cast<double>());
      geometry_msgs::msg::Pose pose_msg = tf2::toMsg(map_to_odom_iso);
      transform.transform.translation.x = pose_msg.position.x;
      transform.transform.translation.y = pose_msg.position.y;
      transform.transform.translation.z = pose_msg.position.z;
      transform.transform.rotation = pose_msg.orientation;
    } else {
      // Publish map -> base_link directly
      transform.header.frame_id = global_frame_id_;
      transform.child_frame_id = robot_frame_id_;
      Eigen::Isometry3d tf_iso(pose.cast<double>());
      geometry_msgs::msg::Pose pose_msg2 = tf2::toMsg(tf_iso);
      transform.transform.translation.x = pose_msg2.position.x;
      transform.transform.translation.y = pose_msg2.position.y;
      transform.transform.translation.z = pose_msg2.position.z;
      transform.transform.rotation = pose_msg2.orientation;
    }

    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      last_tf_msg_ = transform;
      has_valid_tf_ = true;
    }
    broadcaster_->sendTransform(transform);
  }
}

void ScanMatchingComponent::republishTF()
{
  std::lock_guard<std::mutex> lock(tf_mutex_);
  if (!has_valid_tf_) {
    return;
  }
  // Re-broadcast last known map->odom with current timestamp
  last_tf_msg_.header.stamp = this->now();
  broadcaster_->sendTransform(last_tf_msg_);
}

void ScanMatchingComponent::mapArrayCallback(const slam_msgs::msg::MapArray::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received modified map array from backend (%zu submaps)",
    msg->submaps.size());

  if (msg->submaps.size() != submaps_.size()) {
    RCLCPP_WARN(get_logger(),
      "Modified map array size (%zu) != local submaps size (%zu), ignoring",
      msg->submaps.size(), submaps_.size());
    return;
  }

  // Update all submap poses from the backend-corrected map
  for (size_t i = 0; i < msg->submaps.size(); ++i) {
    Eigen::Affine3d affine;
    tf2::fromMsg(msg->submaps[i].pose, affine);
    submaps_[i].pose = affine.matrix().cast<float>();
  }

  // Update current pose to the latest corrected pose
  if (!submaps_.empty()) {
    // The current pose should be adjusted based on the correction to the last submap
    Eigen::Matrix4f last_corrected = submaps_.back().pose;
    current_pose_ = last_corrected;
    previous_pose_ = last_corrected;
  }

  is_map_updated_by_backend_ = true;

  RCLCPP_INFO(get_logger(), "Updated submap poses from backend graph optimization");
}

void ScanMatchingComponent::mapPublishTimerCallback()
{
  std::lock_guard<std::mutex> lock(slam_mutex_);
  if (submaps_.empty()) {
    return;
  }

  // Build and publish full map for visualization
  PointCloudT::Ptr full_map(new PointCloudT);
  for (const auto & submap : submaps_) {
    PointCloudT::Ptr transformed(new PointCloudT);
    pcl::transformPointCloud(*submap.cloud, *transformed, submap.pose);
    *full_map += *transformed;
  }

  // Downsample for visualization
  PointCloudT::Ptr map_downsampled(new PointCloudT);
  applyVoxelFilter(full_map, map_downsampled, vg_size_for_map_);

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_downsampled, map_msg);
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = global_frame_id_;
  map_pub_->publish(map_msg);
}

void ScanMatchingComponent::applyVoxelFilter(
  PointCloudT::Ptr input, PointCloudT::Ptr output, double leaf_size) const
{
  if (leaf_size <= 0.0) {
    *output = *input;
    return;
  }
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*output);
}

void ScanMatchingComponent::filterByRange(PointCloudT::Ptr cloud) const
{
  PointCloudT::Ptr filtered(new PointCloudT);
  filtered->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (dist >= scan_min_range_ && dist <= scan_max_range_) {
      filtered->points.push_back(pt);
    }
  }
  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;
  cloud->swap(*filtered);
}

}  // namespace scanmatching_slam
