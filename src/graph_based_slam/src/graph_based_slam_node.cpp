// Copyright 2024 graph_based_slam Authors
#include <rclcpp/rclcpp.hpp>
#include "graph_based_slam/graph_based_slam_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<graph_based_slam::GraphBasedSlamComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
