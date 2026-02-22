#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/voxel_grid.h>

class SimpleMapping : public rclcpp::Node {
public:
    SimpleMapping() : Node("simple_mapping") {
        map_cloud = std::make_shared<pcl::PCLPointCloud2>();
        
        sub_cam = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "camera/depth_registered/points", 10, 
            std::bind(&SimpleMapping::callback, this, std::placeholders::_1));
        pub_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pcd", 10);
        
        RCLCPP_INFO(this->get_logger(), "Simple Mapping Node Active");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PCLPointCloud2::Ptr incoming(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *incoming);
        
        // Gabungkan ke peta global 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_map(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        pcl::fromPCLPointCloud2(*map_cloud, *current_map);
        pcl::fromPCLPointCloud2(*incoming, *new_cloud);
        
        *current_map += *new_cloud;
        
        // Voxel Grid Filter untuk optimasi 
        pcl::toPCLPointCloud2(*current_map, *map_cloud);
        pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
        vg.setInputCloud(map_cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        vg.filter(*map_cloud);
        
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl_conversions::fromPCL(*map_cloud, out_msg);
        out_msg.header.frame_id = "map";
        pub_map->publish(out_msg);
    }
    
    std::shared_ptr<pcl::PCLPointCloud2> map_cloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMapping>());
    rclcpp::shutdown();
    return 0;
}
