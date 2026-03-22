#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

/**
 * Localization Node - Lokalisasi menggunakan pre-built map (PCD file)
 *
 * Node ini melakukan:
 * 1. Load peta PCD yang sudah dibangun SLAM
 * 2. Scan matching (GICP/NDT) current scan vs map
 * 3. Broadcast TF map → odom
 * 4. Publish pose hasil lokalisasi
 *
 * Cocok untuk navigasi setelah map sudah jadi.
 */
class RgbdCamLoc : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    RgbdCamLoc() : Node("rgbd_cam_loc") {
        // Parameters
        this->declare_parameter("pcd_path", "");
        this->declare_parameter("voxel_leaf_size", 0.06);
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("gicp_max_corr_dist", 1.5);
        this->declare_parameter("gicp_max_iterations", 50);
        this->declare_parameter("gicp_fitness_threshold", 1.0);
        this->declare_parameter("max_cam_d", 8.0);
        this->declare_parameter("crop_min_z", -0.5);
        this->declare_parameter("crop_max_z", 3.0);
        this->declare_parameter("self_filter_size", 0.6);
        this->declare_parameter("sensor_height", 0.4);
        this->declare_parameter("use_ndt", false);
        this->declare_parameter("publish_rate_hz", 10.0);
        this->declare_parameter("loc_update_min_dist", 0.1);

        // Camera positions (sama dengan SLAM node)
        this->declare_parameter("cam1_x", 0.4);
        this->declare_parameter("cam1_y", 0.0);
        this->declare_parameter("cam1_yaw", 0.0);
        this->declare_parameter("cam2_x", -0.4);
        this->declare_parameter("cam2_y", 0.0);
        this->declare_parameter("cam2_yaw", 3.14159);
        this->declare_parameter("cam3_x", 0.0);
        this->declare_parameter("cam3_y", 0.25);
        this->declare_parameter("cam3_yaw", 1.5708);
        this->declare_parameter("cam4_x", 0.0);
        this->declare_parameter("cam4_y", -0.25);
        this->declare_parameter("cam4_yaw", -1.5708);

        loadParameters();

        // Load map
        map_cloud_.reset(new PointCloudT());
        if (pcd_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_path' belum diset! Lokalisasi tidak bisa dimulai.");
            return;
        }

        if (pcl::io::loadPCDFile<PointT>(pcd_path_, *map_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Gagal load map: %s", pcd_path_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Map loaded: %s (%zu points)", pcd_path_.c_str(), map_cloud_->size());

        // Downsample map
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(map_cloud_);
        vg.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
        vg.filter(*map_cloud_);
        RCLCPP_INFO(this->get_logger(), "Map setelah downsample: %zu points", map_cloud_->size());

        map_loaded_ = true;

        // Camera buffers
        c2_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        c3_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        c4_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        // Subscriptions
        sub_c1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in1/points", rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::cam1Callback, this, std::placeholders::_1));
        sub_c2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in2/points", rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::cam2Callback, this, std::placeholders::_1));
        sub_c3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in3/points", rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::cam3Callback, this, std::placeholders::_1));
        sub_c4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in4/points", rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::cam4Callback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::odomCallback, this, std::placeholders::_1));

        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&RgbdCamLoc::initialPoseCallback, this, std::placeholders::_1));

        // Publishers
        pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loc/map_cloud", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/loc/pose", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/loc/trajectory", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer untuk publish map periodically
        map_pub_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&RgbdCamLoc::publishMapTimer, this));

        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(),
            "=== LOCALIZATION READY === Map: %s", pcd_path_.c_str());
    }

private:
    void loadParameters() {
        pcd_path_ = this->get_parameter("pcd_path").as_string();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        map_voxel_leaf_size_ = this->get_parameter("map_voxel_leaf_size").as_double();
        gicp_max_corr_dist_ = this->get_parameter("gicp_max_corr_dist").as_double();
        gicp_max_iterations_ = this->get_parameter("gicp_max_iterations").as_int();
        gicp_fitness_threshold_ = this->get_parameter("gicp_fitness_threshold").as_double();
        max_cam_d_ = this->get_parameter("max_cam_d").as_double();
        crop_min_z_ = this->get_parameter("crop_min_z").as_double();
        crop_max_z_ = this->get_parameter("crop_max_z").as_double();
        self_filter_size_ = this->get_parameter("self_filter_size").as_double();
        sensor_height_ = this->get_parameter("sensor_height").as_double();
        use_ndt_ = this->get_parameter("use_ndt").as_bool();
        loc_update_min_dist_ = this->get_parameter("loc_update_min_dist").as_double();

        cam1_x_ = this->get_parameter("cam1_x").as_double();
        cam1_y_ = this->get_parameter("cam1_y").as_double();
        cam1_yaw_ = this->get_parameter("cam1_yaw").as_double();
        cam2_x_ = this->get_parameter("cam2_x").as_double();
        cam2_y_ = this->get_parameter("cam2_y").as_double();
        cam2_yaw_ = this->get_parameter("cam2_yaw").as_double();
        cam3_x_ = this->get_parameter("cam3_x").as_double();
        cam3_y_ = this->get_parameter("cam3_y").as_double();
        cam3_yaw_ = this->get_parameter("cam3_yaw").as_double();
        cam4_x_ = this->get_parameter("cam4_x").as_double();
        cam4_y_ = this->get_parameter("cam4_y").as_double();
        cam4_yaw_ = this->get_parameter("cam4_yaw").as_double();
    }

    Eigen::Affine3f makeCameraTransform(float x, float y, float yaw) {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.translation() << x, y, sensor_height_;
        t.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        return t;
    }

    PointCloudT::Ptr mergeAndFilter(const sensor_msgs::msg::PointCloud2::SharedPtr& msg_c1) {
        auto merged = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        auto c1 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg_c1, *c1);
        pcl::transformPointCloud(*c1, *c1, makeCameraTransform(cam1_x_, cam1_y_, cam1_yaw_));
        *merged += *c1;

        {
            std::lock_guard<std::mutex> lock(cam_mutex_);
            if (!c2_->empty()) {
                pcl::PointCloud<pcl::PointXYZ> temp;
                pcl::transformPointCloud(*c2_, temp, makeCameraTransform(cam2_x_, cam2_y_, cam2_yaw_));
                *merged += temp;
            }
            if (!c3_->empty()) {
                pcl::PointCloud<pcl::PointXYZ> temp;
                pcl::transformPointCloud(*c3_, temp, makeCameraTransform(cam3_x_, cam3_y_, cam3_yaw_));
                *merged += temp;
            }
            if (!c4_->empty()) {
                pcl::PointCloud<pcl::PointXYZ> temp;
                pcl::transformPointCloud(*c4_, temp, makeCameraTransform(cam4_x_, cam4_y_, cam4_yaw_));
                *merged += temp;
            }
        }

        // Voxel
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(merged);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        vg.filter(*merged);

        // Crop box
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(merged);
        crop.setMin(Eigen::Vector4f(-max_cam_d_, -max_cam_d_, crop_min_z_, 1.0));
        crop.setMax(Eigen::Vector4f(max_cam_d_, max_cam_d_, crop_max_z_, 1.0));
        crop.filter(*merged);

        // Self filter
        pcl::CropBox<pcl::PointXYZ> self;
        self.setInputCloud(merged);
        self.setMin(Eigen::Vector4f(-self_filter_size_, -self_filter_size_, 0.1, 1.0));
        self.setMax(Eigen::Vector4f(self_filter_size_, self_filter_size_, crop_max_z_, 1.0));
        self.setNegative(true);
        self.filter(*merged);

        // Konversi
        auto result = pcl::make_shared<PointCloudT>();
        pcl::copyPointCloud(*merged, *result);
        for (auto& p : result->points) p.intensity = 255;
        return result;
    }

    void cam1Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!map_loaded_ || msg->width * msg->height == 0 || !odom_received_) return;

        auto current_scan = mergeAndFilter(msg);
        if (current_scan->size() < 50) return;

        // Transform ke odom frame
        Eigen::Affine3f odom_tf = Eigen::Affine3f::Identity();
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_tf.translation() << robot_x_, robot_y_, robot_z_;
            odom_tf.rotate(Eigen::Quaternionf(robot_qw_, robot_qx_, robot_qy_, robot_qz_));
        }

        PointCloudT::Ptr cloud_in_odom(new PointCloudT());
        pcl::transformPointCloud(*current_scan, *cloud_in_odom, odom_tf);

        // Cek minimal movement sebelum update
        Eigen::Vector3f current_pos(odom_tf.translation());
        if (loc_initialized_ && (current_pos - last_loc_pos_).norm() < loc_update_min_dist_) {
            // Tetap publish TF lama
            publishTF(msg->header.stamp);
            return;
        }

        // Scan matching vs map
        PointCloudT::Ptr aligned(new PointCloudT());

        if (use_ndt_) {
            pcl::NormalDistributionsTransform<PointT, PointT> ndt;
            ndt.setInputSource(cloud_in_odom);
            ndt.setInputTarget(map_cloud_);
            ndt.setMaximumIterations(gicp_max_iterations_);
            ndt.align(*aligned, map_to_odom_);

            if (ndt.hasConverged() && ndt.getFitnessScore() < gicp_fitness_threshold_) {
                map_to_odom_ = ndt.getFinalTransformation();
                loc_initialized_ = true;
                last_loc_pos_ = current_pos;
            }
        } else {
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
            gicp.setInputSource(cloud_in_odom);
            gicp.setInputTarget(map_cloud_);
            gicp.setMaxCorrespondenceDistance(gicp_max_corr_dist_);
            gicp.setMaximumIterations(gicp_max_iterations_);
            gicp.align(*aligned, map_to_odom_);

            if (gicp.hasConverged() && gicp.getFitnessScore() < gicp_fitness_threshold_) {
                map_to_odom_ = gicp.getFinalTransformation();
                loc_initialized_ = true;
                last_loc_pos_ = current_pos;

                RCLCPP_DEBUG(this->get_logger(), "Localization updated. Fitness: %.4f", gicp.getFitnessScore());
            }
        }

        publishTF(msg->header.stamp);

        // Publish pose
        Eigen::Matrix4f pose_map = map_to_odom_ * odom_tf.matrix();
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = msg->header.stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = pose_map(0, 3);
        ps.pose.position.y = pose_map(1, 3);
        ps.pose.position.z = pose_map(2, 3);
        Eigen::Quaternionf q(Eigen::Matrix3f(pose_map.block<3,3>(0,0)));
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        pub_pose_->publish(ps);

        path_msg_.poses.push_back(ps);
        pub_path_->publish(path_msg_);
    }

    void cam2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        pcl::fromROSMsg(*msg, *c2_);
    }
    void cam3Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        pcl::fromROSMsg(*msg, *c3_);
    }
    void cam4Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        pcl::fromROSMsg(*msg, *c4_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_z_ = msg->pose.pose.position.z;
        robot_qx_ = msg->pose.pose.orientation.x;
        robot_qy_ = msg->pose.pose.orientation.y;
        robot_qz_ = msg->pose.pose.orientation.z;
        robot_qw_ = msg->pose.pose.orientation.w;
        odom_received_ = true;
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Initial pose received!");
        Eigen::Affine3f init_pose = Eigen::Affine3f::Identity();
        init_pose.translation() << msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z;
        Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
        init_pose.rotate(q);
        map_to_odom_ = init_pose.matrix();
        loc_initialized_ = true;
    }

    void publishTF(const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        Eigen::Affine3f aff(map_to_odom_);
        Eigen::Quaternionf q(aff.rotation());
        t.transform.translation.x = aff.translation().x();
        t.transform.translation.y = aff.translation().y();
        t.transform.translation.z = aff.translation().z();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    void publishMapTimer() {
        if (!map_loaded_ || map_cloud_->empty()) return;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*map_cloud_, output);
        output.header.stamp = this->now();
        output.header.frame_id = "map";
        pub_map_cloud_->publish(output);
    }

    // Parameters
    std::string pcd_path_;
    double voxel_leaf_size_, map_voxel_leaf_size_;
    double gicp_max_corr_dist_;
    int gicp_max_iterations_;
    double gicp_fitness_threshold_;
    double max_cam_d_, crop_min_z_, crop_max_z_;
    double self_filter_size_, sensor_height_;
    bool use_ndt_;
    double loc_update_min_dist_;
    double cam1_x_, cam1_y_, cam1_yaw_;
    double cam2_x_, cam2_y_, cam2_yaw_;
    double cam3_x_, cam3_y_, cam3_yaw_;
    double cam4_x_, cam4_y_, cam4_yaw_;

    // State
    bool map_loaded_ = false;
    bool odom_received_ = false;
    bool loc_initialized_ = false;
    Eigen::Matrix4f map_to_odom_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f last_loc_pos_ = Eigen::Vector3f::Zero();

    float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
    float robot_qx_ = 0, robot_qy_ = 0, robot_qz_ = 0, robot_qw_ = 1;
    std::mutex odom_mutex_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr c2_, c3_, c4_;
    std::mutex cam_mutex_;

    PointCloudT::Ptr map_cloud_;
    nav_msgs::msg::Path path_msg_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_c1_, sub_c2_, sub_c3_, sub_c4_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdCamLoc>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
