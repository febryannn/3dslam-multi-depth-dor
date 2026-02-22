#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// Note: sensor_msgs/msg/PointCloud (v1) is legacy in ROS 2. 
// Standard ROS 2 uses PointCloud2 exclusively.

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("point_cloud_converter") {
        pub_points2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2_out", 10);
        sub_points2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points2_in", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // ROS 2 passthrough example 
                pub_points2->publish(*msg);
            });
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points2;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points2;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
