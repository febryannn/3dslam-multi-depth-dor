#include <memory>
#include <chrono>
#include <iostream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <octomap/octomap.h>

#define TO_RAD 0.01745329252

class SlamRgbdCam : public rclcpp::Node {
public:
    SlamRgbdCam() : Node("slam_rgbd_cam") {
        this->declare_parameter("max_cam_d", 8.0);
        max_cam_d = this->get_parameter("max_cam_d").as_double();

        global_map.reset(new pcl::PointCloud<pcl::PointXYZI>());
        octo_map = std::make_shared<octomap::OcTree>(0.1); 

        c2.reset(new pcl::PointCloud<pcl::PointXYZ>());
        c3.reset(new pcl::PointCloud<pcl::PointXYZ>());
        c4.reset(new pcl::PointCloud<pcl::PointXYZ>());

        // Berlangganan langsung ke 4 Kamera Gazebo
        sub_c1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in1/points", rclcpp::SensorDataQoS(), std::bind(&SlamRgbdCam::cam1_main_callback, this, std::placeholders::_1));
        sub_c2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in2/points", rclcpp::SensorDataQoS(), std::bind(&SlamRgbdCam::cam2_callback, this, std::placeholders::_1));
        sub_c3 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in3/points", rclcpp::SensorDataQoS(), std::bind(&SlamRgbdCam::cam3_callback, this, std::placeholders::_1));
        sub_c4 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in4/points", rclcpp::SensorDataQoS(), std::bind(&SlamRgbdCam::cam4_callback, this, std::placeholders::_1));
        
        // Sinkronisasi Odometri langsung dari Gazebo
        sub_robot_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(), std::bind(&SlamRgbdCam::robot_odom_callback, this, std::placeholders::_1));

        pub_cam_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 10);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "[SLAM READY] Gazebo Sync + 360 Scan + GICP + OctoMap Raycasting!");
    }

private:
    void perform_raycasting(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_map, Eigen::Matrix4f map_to_odom, Eigen::Affine3f odom_to_base) {
        Eigen::Vector4f sensor_base(0.0, 0.0, 0.4, 1.0); 
        Eigen::Vector4f sensor_odom = odom_to_base * sensor_base;
        Eigen::Vector4f sensor_map = map_to_odom * sensor_odom;
        octomap::point3d origin(sensor_map.x(), sensor_map.y(), sensor_map.z());

        octomap::Pointcloud octo_cloud;
        for (const auto& pt : cloud_in_map->points) {
            octo_cloud.push_back(pt.x, pt.y, pt.z);
        }

        octo_map->insertPointCloud(octo_cloud, origin, max_cam_d, false, true);

        global_map->clear();
        for(octomap::OcTree::leaf_iterator it = octo_map->begin_leafs(), end=octo_map->end_leafs(); it!= end; ++it) {
            if (octo_map->isNodeOccupied(*it)) {
                pcl::PointXYZI pt;
                pt.x = it.getX(); pt.y = it.getY(); pt.z = it.getZ();
                pt.intensity = 255;
                global_map->push_back(pt);
            }
        }
    }

    void cam2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pcl::fromROSMsg(*msg, *c2); }
    void cam3_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pcl::fromROSMsg(*msg, *c3); }
    void cam4_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pcl::fromROSMsg(*msg, *c4); }

    void cam1_main_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->width * msg->height == 0 || !odom_received) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Merge Cam 1 (Depan)
        pcl::PointCloud<pcl::PointXYZ>::Ptr c1_pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *c1_pc);
        Eigen::Affine3f t1 = Eigen::Affine3f::Identity();
        t1.translation() << 0.4, 0.0, 0.4;
        pcl::transformPointCloud(*c1_pc, *c1_pc, t1);
        *merged += *c1_pc;

        // Merge Cam 2 (Belakang)
        if (!c2->empty()) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            Eigen::Affine3f t2 = Eigen::Affine3f::Identity();
            t2.translation() << -0.4, 0.0, 0.4;
            t2.rotate(Eigen::AngleAxisf(3.14159, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(*c2, temp, t2);
            *merged += temp;
        }

        // Merge Cam 3 (Kiri)
        if (!c3->empty()) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            Eigen::Affine3f t3 = Eigen::Affine3f::Identity();
            t3.translation() << 0.0, 0.25, 0.4;
            t3.rotate(Eigen::AngleAxisf(1.5708, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(*c3, temp, t3);
            *merged += temp;
        }

        // Merge Cam 4 (Kanan)
        if (!c4->empty()) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            Eigen::Affine3f t4 = Eigen::Affine3f::Identity();
            t4.translation() << 0.0, -0.25, 0.4;
            t4.rotate(Eigen::AngleAxisf(-1.5708, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(*c4, temp, t4);
            *merged += temp;
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(merged);
        vg.setLeafSize(0.06f, 0.06f, 0.06f);
        vg.filter(*merged);

        pcl::CropBox<pcl::PointXYZ> room_filter;
        room_filter.setInputCloud(merged);
        room_filter.setMin(Eigen::Vector4f(-max_cam_d, -max_cam_d, -0.5, 1.0)); // Tangkap Lantai
        room_filter.setMax(Eigen::Vector4f(max_cam_d, max_cam_d, 2.0, 1.0));
        room_filter.filter(*merged);

        pcl::CropBox<pcl::PointXYZ> dyn_eraser;
        dyn_eraser.setInputCloud(merged);
        dyn_eraser.setMin(Eigen::Vector4f(-1.5, -1.5, 0.1, 1.0));
        dyn_eraser.setMax(Eigen::Vector4f(1.5, 1.5, 2.0, 1.0));
        dyn_eraser.setNegative(true);
        dyn_eraser.filter(*merged);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(merged);
        sor.setMeanK(30);
        sor.setStddevMulThresh(1.0);
        sor.filter(*merged);

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*merged, *current_cloud);
        for (auto &pt : current_cloud->points) pt.intensity = 255;

        Eigen::Affine3f transform_odom_base = Eigen::Affine3f::Identity();
        transform_odom_base.translation() << robotx, roboty, robotz;
        transform_odom_base.rotate(Eigen::Quaternionf(robotqw, robotqx, robotqy, robotqz));

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_odom(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*current_cloud, *cloud_in_odom, transform_odom_base);

        if (is_first_frame) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_map(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud_in_odom, *cloud_in_map, map_to_odom_transform);
            perform_raycasting(cloud_in_map, map_to_odom_transform, transform_odom_base);
            is_first_frame = false;
        } else {
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
            gicp.setInputSource(cloud_in_odom);
            gicp.setInputTarget(global_map);
            gicp.setMaxCorrespondenceDistance(0.5); 
            gicp.setMaximumIterations(35);
            gicp.setTransformationEpsilon(1e-6);

            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            gicp.align(*aligned_cloud, map_to_odom_transform); 

            if (gicp.hasConverged()) {
                map_to_odom_transform = gicp.getFinalTransformation();
                perform_raycasting(aligned_cloud, map_to_odom_transform, transform_odom_base);
            }
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*global_map, output);
        output.header.stamp = msg->header.stamp;
        output.header.frame_id = "map"; 
        pub_cam_filter->publish(output);

        // TF MAP -> ODOM
        geometry_msgs::msg::TransformStamped t_map;
        t_map.header.stamp = msg->header.stamp;
        t_map.header.frame_id = "map";
        t_map.child_frame_id = "odom";
        Eigen::Affine3f affine_map(map_to_odom_transform);
        Eigen::Quaternionf q_map(affine_map.rotation());
        t_map.transform.translation.x = affine_map.translation().x();
        t_map.transform.translation.y = affine_map.translation().y();
        t_map.transform.translation.z = affine_map.translation().z();
        t_map.transform.rotation.x = q_map.x();
        t_map.transform.rotation.y = q_map.y();
        t_map.transform.rotation.z = q_map.z();
        t_map.transform.rotation.w = q_map.w();
        tf_broadcaster->sendTransform(t_map);
    }

    void robot_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robotx = msg->pose.pose.position.x;
        roboty = msg->pose.pose.position.y;
        robotz = msg->pose.pose.position.z;
        robotqx = msg->pose.pose.orientation.x;
        robotqy = msg->pose.pose.orientation.y;
        robotqz = msg->pose.pose.orientation.z;
        robotqw = msg->pose.pose.orientation.w;
        odom_received = true;
    }

    double max_cam_d;
    float robotx = 0, roboty = 0, robotz = 0;
    float robotqx = 0, robotqy = 0, robotqz = 0, robotqw = 1;
    bool odom_received = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map;
    std::shared_ptr<octomap::OcTree> octo_map; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr c2, c3, c4;

    bool is_first_frame = true;
    Eigen::Matrix4f map_to_odom_transform = Eigen::Matrix4f::Identity();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_c1, sub_c2, sub_c3, sub_c4;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_odom;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cam_filter;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamRgbdCam>());
    rclcpp::shutdown();
    return 0;
}
