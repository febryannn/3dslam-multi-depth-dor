#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomToUdpRelay : public rclcpp::Node {
public:
    OdomToUdpRelay() : Node("odom_to_udp_relay") {
        sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&OdomToUdpRelay::cb, this, std::placeholders::_1));
        pub_ = create_publisher<udp_bot_msgs::msg::TerimaUdp>("data_terima_udp", 10);
    }
private:
    void cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        udp_bot_msgs::msg::TerimaUdp out;
        out.posisi_x_buffer = msg->pose.pose.position.x;
        out.posisi_y_buffer = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y);
        out.sudut_w_buffer = y * 180.0 / M_PI; // Konversi ke derajat untuk SLAM
        pub_->publish(out);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<udp_bot_msgs::msg::TerimaUdp>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToUdpRelay>());
    rclcpp::shutdown();
    return 0;
}
