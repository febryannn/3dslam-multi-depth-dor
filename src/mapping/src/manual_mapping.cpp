#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mapping/srv/input_pos.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

using namespace std::chrono_literals;

class ManualMapping : public rclcpp::Node {
public:
    ManualMapping() : Node("manual_mapping") {
        // Parametrisasi 
        this->declare_parameter("maxf_points_out", 1.0);
        this->declare_parameter("min_cam_d", 0.6);
        this->declare_parameter("max_cam_d", 8.0);

        freq_points = this->get_parameter("maxf_points_out").as_double();
        min_cam_d = this->get_parameter("min_cam_d").as_double();
        max_cam_d = this->get_parameter("max_cam_d").as_double();

        // Publisher & Subscriber
        sub_cam_raw = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "full_pointcloud", 10, std::bind(&ManualMapping::cam_filter_callback, this, std::placeholders::_1));
        pub_cam_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("cam_pcd", 10);

        // Service & TF
        update_pos_srv = this->create_service<mapping::srv::InputPos>(
            "update_pos", std::bind(&ManualMapping::update_pos_callback, this, std::placeholders::_1, std::placeholders::_2));
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timers
        auto timer_period = std::chrono::duration<double>(1.0 / freq_points);
        pub_timer = this->create_wall_timer(timer_period, std::bind(&ManualMapping::pub_cam_filter_callback, this));
        robot_timer = this->create_wall_timer(10ms, std::bind(&ManualMapping::robot_pos_callback, this));

        RCLCPP_INFO(this->get_logger(), "Manual Mapping Ready");
    }

private:
    void cam_filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        input_msg = *msg;
        status_pub = true;
    }

    void pub_cam_filter_callback() {
        if (!status_pub) return;

        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(input_msg, *cloud);

        // Filtering Logic 
        pcl::CropBox<pcl::PCLPointCloud2> cb;
        cb.setInputCloud(cloud);
        cb.setMin(Eigen::Vector4f(min_cam_d, -10.0, -10.0, 1.0));
        cb.setMax(Eigen::Vector4f(max_cam_d, 10.0, 10.0, 1.0));
        cb.filter(*cloud);

        pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*cloud);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl_conversions::fromPCL(*cloud, out_msg);
        out_msg.header = input_msg.header;
        pub_cam_filter->publish(out_msg);
        status_pub = false;
    }

    void update_pos_callback(const std::shared_ptr<mapping::srv::InputPos::Request> req,
                             std::shared_ptr<mapping::srv::InputPos::Response> res) {
        robot_x = req->posx;
        robot_y = req->posy;
        robot_w = req->posw;
        res->status_pos = true;
    }

    void robot_pos_callback() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = robot_x;
        t.transform.translation.y = robot_y;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_w * 0.0174533);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster->sendTransform(t);
    }

    bool status_pub = false;
    float robot_x = 0, robot_y = 0, robot_w = 0;
    double freq_points, min_cam_d, max_cam_d;
    sensor_msgs::msg::PointCloud2 input_msg;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam_raw;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cam_filter;
    rclcpp::Service<mapping::srv::InputPos>::SharedPtr update_pos_srv;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr pub_timer, robot_timer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualMapping>());
    rclcpp::shutdown();
    return 0;
}
