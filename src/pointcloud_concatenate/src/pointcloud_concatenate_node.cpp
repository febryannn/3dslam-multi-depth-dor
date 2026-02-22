#include "pointcloud_concatenate/pointcloud_concatenate.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointcloudConcatenate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
