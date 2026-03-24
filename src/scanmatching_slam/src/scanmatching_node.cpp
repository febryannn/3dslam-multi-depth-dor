// Copyright 2024 scanmatching_slam Authors
#include <rclcpp/rclcpp.hpp>
#include "scanmatching_slam/scanmatching_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<scanmatching_slam::ScanMatchingComponent>(options);
  // Use multi-threaded executor so timer callbacks (TF republish, map publish)
  // can fire even while NDT scan matching blocks the cloud callback
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
