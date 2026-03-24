// Copyright 2024 scanmatching_slam Authors
#include <rclcpp/rclcpp.hpp>
#include "scanmatching_slam/scanmatching_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<scanmatching_slam::ScanMatchingComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
