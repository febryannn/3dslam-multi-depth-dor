#include "pointcloud_concatenate/pointcloud_concatenate.hpp"

PointcloudConcatenate::PointcloudConcatenate() : Node("pointcloud_concatenate") {
    cloud_in1_received_ = cloud_in2_received_ = cloud_in3_received_ = cloud_in4_received_ = false;
    
    handleParams();
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    sub_cloud_in1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in1", 1, std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1));
    sub_cloud_in2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in2", 1, std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1));
    sub_cloud_in3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in3", 1, std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1));
    sub_cloud_in4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in4", 1, std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1));
    
    // Nama topik disamakan dengan input slam_rgbd_cam_node
    pub_cloud_out_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("full_pointcloud", 1);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / param_hz_)),
        std::bind(&PointcloudConcatenate::update, this));
}

PointcloudConcatenate::~PointcloudConcatenate() {}

void PointcloudConcatenate::handleParams() {
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("clouds", 4);
    this->declare_parameter("hz", 10.0);
    param_frame_target_ = this->get_parameter("target_frame").as_string();
    param_clouds_ = this->get_parameter("clouds").as_int();
    param_hz_ = this->get_parameter("hz").as_double();
}

// Callbacks (X4)
void PointcloudConcatenate::subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_in1_ = *msg; cloud_in1_received_ = true; }
void PointcloudConcatenate::subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_in2_ = *msg; cloud_in2_received_ = true; }
void PointcloudConcatenate::subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_in3_ = *msg; cloud_in3_received_ = true; }
void PointcloudConcatenate::subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_in4_ = *msg; cloud_in4_received_ = true; }

void PointcloudConcatenate::update() {
    if (pub_cloud_out_->get_subscription_count() == 0 || param_clouds_ < 1) return;

    sensor_msgs::msg::PointCloud2 merged_msg; // Gunakan tipe ROS langsung untuk menghindari error lambda
    bool first_processed = false;

    auto process = [&](const sensor_msgs::msg::PointCloud2& in, int id) {
        try {
            sensor_msgs::msg::PointCloud2 trans;
            auto t = tf_buffer_->lookupTransform(param_frame_target_, in.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(in, trans, t);
            
            if (!first_processed) { merged_msg = trans; first_processed = true; }
            else { pcl::concatenatePointCloud(merged_msg, trans, merged_msg); } // Memanggil fungsi pcl_conversions
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Cloud %d transform error: %s", id, ex.what());
        }
    };

    if (param_clouds_ >= 1 && cloud_in1_received_) process(cloud_in1_, 1);
    if (param_clouds_ >= 2 && cloud_in2_received_) process(cloud_in2_, 2);
    if (param_clouds_ >= 3 && cloud_in3_received_) process(cloud_in3_, 3);
    if (param_clouds_ >= 4 && cloud_in4_received_) process(cloud_in4_, 4);

    if (first_processed) {
        merged_msg.header.stamp = this->now();
        merged_msg.header.frame_id = param_frame_target_;
        pub_cloud_out_->publish(merged_msg);
    }
}
