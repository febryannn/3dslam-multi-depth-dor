// Copyright 2024 graph_based_slam Authors
// Adapted from lidarslam_ros2 graph_based_slam_component
#include "graph_based_slam/graph_based_slam_component.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <chrono>
#include <memory>

namespace graph_based_slam
{

GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: Node("graph_based_slam", options),
  tfbuffer_(this->get_clock()),
  listener_(tfbuffer_),
  broadcaster_(this),
  clock_(*this->get_clock())
{
  RCLCPP_INFO(get_logger(), "Initializing GraphBasedSlamComponent");

  // Declare parameters
  registration_method_ = this->declare_parameter("registration_method", std::string("NDT"));
  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.2);
  ndt_resolution_ = this->declare_parameter("ndt_resolution", 5.0);
  ndt_num_threads_ = this->declare_parameter("ndt_num_threads", 0);
  loop_detection_period_ = this->declare_parameter("loop_detection_period", 1000);
  threshold_loop_closure_score_ = this->declare_parameter("threshold_loop_closure_score", 1.0);
  distance_loop_closure_ = this->declare_parameter("distance_loop_closure", 20.0);
  range_of_searching_loop_closure_ =
    this->declare_parameter("range_of_searching_loop_closure", 20.0);
  search_submap_num_ = this->declare_parameter("search_submap_num", 3);
  num_adjacent_pose_cnstraints_ = this->declare_parameter("num_adjacent_pose_cnstraints", 5);
  use_save_map_in_loop_ = this->declare_parameter("use_save_map_in_loop", true);
  debug_flag_ = this->declare_parameter("debug_flag", false);

  // Setup registration
  if (registration_method_ == "NDT") {
    auto ndt = pcl::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ,
        pcl::PointXYZ>>();
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads_ > 0) {
      ndt->setNumThreads(ndt_num_threads_);
    }
    registration_ = ndt;
  } else {
    auto gicp = pcl::make_shared<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ,
        pcl::PointXYZ>>();
    if (ndt_num_threads_ > 0) {
      gicp->setNumThreads(ndt_num_threads_);
    }
    registration_ = gicp;
  }

  // Setup voxel grid filter
  voxelgrid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

  initializePubSub();

  RCLCPP_INFO(get_logger(), "GraphBasedSlamComponent initialized");
}

void GraphBasedSlamComponent::initializePubSub()
{
  // Subscriber: map_array from frontend
  map_array_sub_ = this->create_subscription<slam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](const slam_msgs::msg::MapArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mtx_);
      map_array_msg_ = *msg;
      if (!initial_map_array_received_) {
        initial_map_array_received_ = true;
      }
      is_map_array_updated_ = true;
    });

  // Publishers
  modified_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  modified_map_array_pub_ = this->create_publisher<slam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  modified_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "modified_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // Timer for loop closure detection
  loop_detect_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(loop_detection_period_),
    std::bind(&GraphBasedSlamComponent::searchLoop, this));

  // Map save service
  map_save_srv_ = this->create_service<std_srvs::srv::Empty>(
    "map_save",
    [this](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      std::lock_guard<std::mutex> lock(mtx_);
      RCLCPP_INFO(get_logger(), "Map save service called");
      doPoseAdjustment(map_array_msg_, true);
    });
}

void GraphBasedSlamComponent::searchLoop()
{
  if (!initial_map_array_received_) {
    return;
  }
  if (!is_map_array_updated_) {
    return;
  }

  slam_msgs::msg::MapArray map_array_msg;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    map_array_msg = map_array_msg_;
    is_map_array_updated_ = false;
  }

  int num_submaps = static_cast<int>(map_array_msg.submaps.size());
  if (num_submaps < 2) {
    return;
  }

  // Get the latest submap
  auto & latest_submap = map_array_msg.submaps[num_submaps - 1];

  // Convert latest cloud from ROS to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(latest_submap.cloud, *latest_cloud);

  // Transform latest cloud to global frame if cloud_coordinate is LOCAL
  if (map_array_msg.cloud_coordinate == slam_msgs::msg::MapArray::LOCAL) {
    Eigen::Affine3d latest_affine;
    tf2::fromMsg(latest_submap.pose, latest_affine);
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, latest_affine.matrix());
  }

  double latest_moving_distance = latest_submap.distance;

  // Search for loop closure candidates
  double min_dist = std::numeric_limits<double>::max();
  int min_index = -1;

  for (int i = 0; i < num_submaps - 1; i++) {
    auto & submap = map_array_msg.submaps[i];
    // Must have traveled enough distance
    if (latest_moving_distance - submap.distance < distance_loop_closure_) {
      continue;
    }
    // Euclidean distance between poses
    double dx = latest_submap.pose.position.x - submap.pose.position.x;
    double dy = latest_submap.pose.position.y - submap.pose.position.y;
    double dz = latest_submap.pose.position.z - submap.pose.position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist > range_of_searching_loop_closure_) {
      continue;
    }
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }

  if (min_index < 0) {
    return;
  }

  if (debug_flag_) {
    RCLCPP_INFO(
      get_logger(), "Loop closure candidate found: submap %d -> %d (dist: %.2f)",
      num_submaps - 1, min_index, min_dist);
  }

  // Build target cloud from adjacent submaps around the candidate
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (int j = std::max(0, min_index - search_submap_num_);
    j <= std::min(num_submaps - 2, min_index + search_submap_num_); j++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(map_array_msg.submaps[j].cloud, *submap_cloud);

    if (map_array_msg.cloud_coordinate == slam_msgs::msg::MapArray::LOCAL) {
      Eigen::Affine3d submap_affine;
      tf2::fromMsg(map_array_msg.submaps[j].pose, submap_affine);
      pcl::transformPointCloud(*submap_cloud, *submap_cloud, submap_affine.matrix());
    }
    *target_cloud += *submap_cloud;
  }

  // Voxel filter target cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target(new pcl::PointCloud<pcl::PointXYZ>());
  voxelgrid_.setInputCloud(target_cloud);
  voxelgrid_.filter(*filtered_target);

  // Run registration
  registration_->setInputSource(latest_cloud);
  registration_->setInputTarget(filtered_target);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  registration_->align(*output_cloud);

  double fitness_score = registration_->getFitnessScore();

  if (debug_flag_) {
    RCLCPP_INFO(get_logger(), "Loop closure fitness score: %.4f (threshold: %.4f)",
      fitness_score, threshold_loop_closure_score_);
  }

  if (fitness_score > threshold_loop_closure_score_) {
    RCLCPP_INFO(get_logger(), "Loop closure rejected (score too high): %.4f", fitness_score);
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Loop closure accepted: submap %d -> %d (score: %.4f)",
    num_submaps - 1, min_index, fitness_score);

  // Compute relative pose (EXACT from lidarslam_ros2)
  Eigen::Affine3d init_affine;
  tf2::fromMsg(latest_submap.pose, init_affine);
  Eigen::Affine3d submap_affine;
  tf2::fromMsg(map_array_msg.submaps[min_index].pose, submap_affine);

  Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
  Eigen::Isometry3d to = Eigen::Isometry3d(
    registration_->getFinalTransformation().cast<double>() * init_affine.matrix());

  LoopEdge loop_edge;
  loop_edge.pair_id = std::make_pair(min_index, num_submaps - 1);
  loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
  loop_edges_.push_back(loop_edge);

  RCLCPP_INFO(
    get_logger(), "Loop edge added: %d -> %d, total loop edges: %zu",
    min_index, num_submaps - 1, loop_edges_.size());

  doPoseAdjustment(map_array_msg, use_save_map_in_loop_);
}

void GraphBasedSlamComponent::doPoseAdjustment(
  slam_msgs::msg::MapArray map_array_msg, bool do_save_map)
{
  int num_submaps = static_cast<int>(map_array_msg.submaps.size());
  if (num_submaps < 2) {
    return;
  }

  RCLCPP_INFO(get_logger(), "Starting pose graph optimization with %d submaps", num_submaps);

  // Setup g2o optimizer
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
  using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // Add vertices for each submap
  for (int i = 0; i < num_submaps; i++) {
    Eigen::Affine3d affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d iso = Eigen::Isometry3d(affine.matrix());

    auto * vertex = new g2o::VertexSE3();
    vertex->setId(i);
    vertex->setEstimate(iso);
    if (i == 0) {
      vertex->setFixed(true);
    }
    optimizer.addVertex(vertex);
  }

  // Information matrix for edges
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();

  // Add adjacent pose constraints
  for (int i = 1; i < num_submaps; i++) {
    if (i > num_adjacent_pose_cnstraints_) {
      for (int j = 0; j < num_adjacent_pose_cnstraints_; j++) {
        int from_id = i - num_adjacent_pose_cnstraints_ + j;
        int to_id = i;

        Eigen::Affine3d from_affine;
        tf2::fromMsg(map_array_msg.submaps[from_id].pose, from_affine);
        Eigen::Affine3d to_affine;
        tf2::fromMsg(map_array_msg.submaps[to_id].pose, to_affine);

        Eigen::Isometry3d from_iso = Eigen::Isometry3d(from_affine.matrix());
        Eigen::Isometry3d to_iso = Eigen::Isometry3d(to_affine.matrix());
        Eigen::Isometry3d relative_pose = Eigen::Isometry3d(from_iso.inverse() * to_iso);

        auto * edge = new g2o::EdgeSE3();
        edge->setId(static_cast<int>(optimizer.edges().size()));
        edge->setVertex(0, optimizer.vertex(from_id));
        edge->setVertex(1, optimizer.vertex(to_id));
        edge->setMeasurement(relative_pose);
        edge->setInformation(info_mat);
        optimizer.addEdge(edge);
      }
    } else {
      // For early submaps, connect to all previous
      for (int j = 0; j < i; j++) {
        Eigen::Affine3d from_affine;
        tf2::fromMsg(map_array_msg.submaps[j].pose, from_affine);
        Eigen::Affine3d to_affine;
        tf2::fromMsg(map_array_msg.submaps[i].pose, to_affine);

        Eigen::Isometry3d from_iso = Eigen::Isometry3d(from_affine.matrix());
        Eigen::Isometry3d to_iso = Eigen::Isometry3d(to_affine.matrix());
        Eigen::Isometry3d relative_pose = Eigen::Isometry3d(from_iso.inverse() * to_iso);

        auto * edge = new g2o::EdgeSE3();
        edge->setId(static_cast<int>(optimizer.edges().size()));
        edge->setVertex(0, optimizer.vertex(j));
        edge->setVertex(1, optimizer.vertex(i));
        edge->setMeasurement(relative_pose);
        edge->setInformation(info_mat);
        optimizer.addEdge(edge);
      }
    }
  }

  // Add loop closure edges
  for (const auto & loop_edge : loop_edges_) {
    auto * edge = new g2o::EdgeSE3();
    edge->setId(static_cast<int>(optimizer.edges().size()));
    edge->setVertex(0, optimizer.vertex(loop_edge.pair_id.first));
    edge->setVertex(1, optimizer.vertex(loop_edge.pair_id.second));
    edge->setMeasurement(loop_edge.relative_pose);
    edge->setInformation(info_mat);
    optimizer.addEdge(edge);
  }

  // Optimize
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  RCLCPP_INFO(get_logger(), "Pose graph optimization completed");

  // Rebuild submap poses from optimized vertices
  slam_msgs::msg::MapArray modified_map_array;
  modified_map_array.header = map_array_msg.header;
  modified_map_array.header.stamp = this->now();
  modified_map_array.cloud_coordinate = map_array_msg.cloud_coordinate;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>());
  nav_msgs::msg::Path modified_path;
  modified_path.header.frame_id = "map";
  modified_path.header.stamp = this->now();

  for (int i = 0; i < num_submaps; i++) {
    auto * vertex = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Isometry3d optimized_pose = vertex->estimate();
    Eigen::Affine3d optimized_affine(optimized_pose.matrix());

    slam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
    submap.pose = tf2::toMsg(optimized_affine);
    submap.cloud = map_array_msg.submaps[i].cloud;
    submap.distance = map_array_msg.submaps[i].distance;
    modified_map_array.submaps.push_back(submap);

    // Build global map
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(submap.cloud, *submap_cloud);

    if (map_array_msg.cloud_coordinate == slam_msgs::msg::MapArray::LOCAL) {
      pcl::transformPointCloud(*submap_cloud, *submap_cloud, optimized_pose.matrix());
    }
    *global_map += *submap_cloud;

    // Build path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = submap.header.stamp;
    pose_stamped.pose = submap.pose;
    modified_path.poses.push_back(pose_stamped);
  }

  // Publish modified map
  sensor_msgs::msg::PointCloud2 modified_map_msg;
  pcl::toROSMsg(*global_map, modified_map_msg);
  modified_map_msg.header.frame_id = "map";
  modified_map_msg.header.stamp = this->now();
  modified_map_pub_->publish(modified_map_msg);

  // Publish modified map array
  modified_map_array_pub_->publish(modified_map_array);

  // Publish modified path
  modified_path_pub_->publish(modified_path);

  RCLCPP_INFO(get_logger(), "Published optimized map, map_array, and path");

  // Save map if requested
  if (do_save_map) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxelgrid_.setInputCloud(global_map);
    voxelgrid_.filter(*save_cloud);

    std::string save_path = "optimized_map.pcd";
    pcl::io::savePCDFileBinary(save_path, *save_cloud);
    RCLCPP_INFO(get_logger(), "Map saved to: %s (%zu points)",
      save_path.c_str(), save_cloud->size());
  }
}

}  // namespace graph_based_slam
