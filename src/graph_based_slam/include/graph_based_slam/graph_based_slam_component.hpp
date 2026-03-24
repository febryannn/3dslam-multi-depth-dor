#ifndef GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_HPP_
#define GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <slam_msgs/msg/map_array.hpp>
#include <slam_msgs/msg/sub_map.hpp>

#include <Eigen/Dense>

#include <mutex>
#include <utility>
#include <vector>
#include <string>

namespace graph_based_slam
{

class GraphBasedSlamComponent : public rclcpp::Node
{
public:
  explicit GraphBasedSlamComponent(const rclcpp::NodeOptions & options);

private:
  void initializePubSub();
  void mapArrayCallback(const slam_msgs::msg::MapArray::SharedPtr msg);
  void searchLoop();
  void doPoseAdjustment(slam_msgs::msg::MapArray map_array_msg, bool do_save_map);

  struct LoopEdge {
    std::pair<int, int> pair_id;
    Eigen::Isometry3d relative_pose;
  };
  std::vector<LoopEdge> loop_edges_;

  // Subscribers / Publishers
  rclcpp::Subscription<slam_msgs::msg::MapArray>::SharedPtr map_array_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_pub_;
  rclcpp::Publisher<slam_msgs::msg::MapArray>::SharedPtr modified_map_array_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr modified_path_pub_;
  rclcpp::TimerBase::SharedPtr loop_detect_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr map_save_srv_;

  // Registration for loop closure matching
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration_;
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_;

  // State
  slam_msgs::msg::MapArray map_array_msg_;
  bool initial_map_array_received_{false};
  bool is_map_array_updated_{false};
  std::mutex mtx_;

  // TF
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Clock clock_;

  // Parameters
  std::string registration_method_;
  double voxel_leaf_size_;
  double ndt_resolution_;
  int ndt_num_threads_;
  int loop_detection_period_;
  double threshold_loop_closure_score_;
  double distance_loop_closure_;
  double range_of_searching_loop_closure_;
  int search_submap_num_;
  int num_adjacent_pose_cnstraints_;
  bool use_save_map_in_loop_;
  bool debug_flag_;
};

}  // namespace graph_based_slam

#endif  // GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_HPP_
