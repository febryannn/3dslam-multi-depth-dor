#ifndef POINTCLOUD_CONCATENATE_HPP
#define POINTCLOUD_CONCATENATE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

class PointcloudConcatenate : public rclcpp::Node
{
public:
    PointcloudConcatenate();
    ~PointcloudConcatenate();

private:
    void subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    void update();
    void handleParams();
    
    std::string param_frame_target_;
    int param_clouds_;
    double param_hz_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in3_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in4_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::PointCloud2 cloud_in1_, cloud_in2_, cloud_in3_, cloud_in4_;
    bool cloud_in1_received_, cloud_in2_received_, cloud_in3_received_, cloud_in4_received_;
};

#endif
