#include <memory>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define TO_RAD 0.01745329252

class RgbdCamLoc : public rclcpp::Node {
public:
    RgbdCamLoc() : Node("rgbd_cam_loc") {
        this->declare_parameter("pcd_path", "/home/febryan/agv_ros2_ws/src/localization/maps/map.pcd");
        pcd_path = this->get_parameter("pcd_path").as_string();

        pub_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        pub_init_pos = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        // Load Map
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_pcd(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path, *map_pcd) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", pcd_path.c_str());
        } else {
            sensor_msgs::msg::PointCloud2 output_map;
            pcl::toROSMsg(*map_pcd, output_map);
            output_map.header.frame_id = "map";
            
            // Publish map and init pose a few times
            for(int i=0; i<3; i++) {
                pub_map->publish(output_map);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        RCLCPP_INFO(this->get_logger(), "[RGBD CAM LOCALIZATION READY]");
    }

private:
    std::string pcd_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pos;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdCamLoc>());
    rclcpp::shutdown();
    return 0;
}
