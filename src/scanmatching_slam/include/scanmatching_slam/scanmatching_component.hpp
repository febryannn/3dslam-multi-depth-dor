// Copyright 2024 scanmatching_slam Authors
// Adapted from lidarslam_ros2 scanmatching_component
#ifndef SCANMATCHING_SLAM__SCANMATCHING_COMPONENT_HPP_
#define SCANMATCHING_SLAM__SCANMATCHING_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>
#include <pcl_conversions/pcl_conversions.h>

#include <slam_msgs/msg/map_array.hpp>
#include <slam_msgs/msg/sub_map.hpp>

#include <Eigen/Dense>

#include <mutex>
#include <string>
#include <vector>

namespace scanmatching_slam
{

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

class ScanMatchingComponent : public rclcpp::Node
{
public:
  explicit ScanMatchingComponent(const rclcpp::NodeOptions & options);

private:
  // Callbacks
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mapArrayCallback(const slam_msgs::msg::MapArray::SharedPtr msg);

  // Core functions
  void initializeMap(PointCloudT::Ptr cloud, const rclcpp::Time & stamp);
  void receiveCloud(PointCloudT::Ptr cloud, const rclcpp::Time & stamp);
  void updateMap(PointCloudT::Ptr cloud, const Eigen::Matrix4f & pose);
  void publishMapAndPose(const Eigen::Matrix4f & pose, const rclcpp::Time & stamp);

  // Helpers
  Eigen::Matrix4f getOdomInitialGuess();
  PointCloudT::Ptr getTargetCloud() const;
  void applyVoxelFilter(
    PointCloudT::Ptr input, PointCloudT::Ptr output, double leaf_size) const;
  void filterByRange(PointCloudT::Ptr cloud) const;

  // Registration
  pcl::Registration<PointT, PointT>::Ptr registration_;

  // Publishers
  rclcpp::Publisher<slam_msgs::msg::MapArray>::SharedPtr map_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<slam_msgs::msg::MapArray>::SharedPtr modified_map_sub_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  tf2_ros::Buffer tfbuffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  // SubMap storage
  struct SubMapData
  {
    PointCloudT::Ptr cloud;
    Eigen::Matrix4f pose;
    double distance;
  };
  std::vector<SubMapData> submaps_;

  // State
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f previous_pose_ = Eigen::Matrix4f::Identity();
  bool initial_cloud_received_ = false;
  double total_distance_ = 0.0;
  double submap_distance_ = 0.0;
  bool is_map_updated_by_backend_ = false;

  // Odometry
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Odometry::SharedPtr prev_odom_;
  std::mutex odom_mutex_;

  // Path
  nav_msgs::msg::Path path_;

  // Map publish timer
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  void mapPublishTimerCallback();

  // Parameters
  std::string registration_method_;
  double ndt_resolution_;
  int ndt_num_threads_;
  double gicp_corr_dist_;
  double trans_for_mapupdate_;
  double vg_size_for_input_;
  double vg_size_for_map_;
  double scan_min_range_;
  double scan_max_range_;
  int num_targeted_cloud_;
  double map_publish_period_;
  bool use_odom_;
  bool publish_tf_;
  std::string global_frame_id_;
  std::string robot_frame_id_;
  std::string odom_frame_id_;
  std::string input_cloud_topic_;
  std::string odom_topic_;
};

}  // namespace scanmatching_slam

#endif  // SCANMATCHING_SLAM__SCANMATCHING_COMPONENT_HPP_
