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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

/**
 * Localization Node - Lokalisasi menggunakan pre-built map (PCD file)
 *
 * Mengikuti arsitektur lidarslam_ros2:
 * 1. Load peta PCD yang sudah dibangun SLAM
 * 2. Scan matching (GICP/NDT) current scan vs map
 * 3. Broadcast TF map -> odom (via calculateMapToOdomTransform)
 * 4. Publish pose hasil lokalisasi
 */
class RgbdCamLoc : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    RgbdCamLoc() : Node("rgbd_cam_loc"),
        clock_(RCL_ROS_TIME),
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_),
        broadcaster_(this)
    {
        RCLCPP_INFO(get_logger(), "initialization start");

        // Parameters
        declare_parameter("pcd_path", "");
        get_parameter("pcd_path", pcd_path_);
        declare_parameter("global_frame_id", "map");
        get_parameter("global_frame_id", global_frame_id_);
        declare_parameter("robot_frame_id", "base_link");
        get_parameter("robot_frame_id", robot_frame_id_);
        declare_parameter("odom_frame_id", "odom");
        get_parameter("odom_frame_id", odom_frame_id_);
        declare_parameter("registration_method", "GICP");
        get_parameter("registration_method", registration_method_);

        double ndt_resolution;
        double gicp_corr_dist;
        declare_parameter("ndt_resolution", 5.0);
        get_parameter("ndt_resolution", ndt_resolution);
        declare_parameter("gicp_max_corr_dist", 1.5);
        get_parameter("gicp_max_corr_dist", gicp_corr_dist);
        declare_parameter("gicp_max_iterations", 50);
        get_parameter("gicp_max_iterations", gicp_max_iterations_);
        declare_parameter("gicp_fitness_threshold", 1.0);
        get_parameter("gicp_fitness_threshold", gicp_fitness_threshold_);

        declare_parameter("voxel_leaf_size", 0.06);
        get_parameter("voxel_leaf_size", voxel_leaf_size_);
        declare_parameter("map_voxel_leaf_size", 0.1);
        get_parameter("map_voxel_leaf_size", map_voxel_leaf_size_);
        declare_parameter("use_min_max_filter", true);
        get_parameter("use_min_max_filter", use_min_max_filter_);
        declare_parameter("scan_min_range", 0.3);
        get_parameter("scan_min_range", scan_min_range_);
        declare_parameter("scan_max_range", 8.0);
        get_parameter("scan_max_range", scan_max_range_);
        declare_parameter("loc_update_min_dist", 0.1);
        get_parameter("loc_update_min_dist", loc_update_min_dist_);

        declare_parameter("publish_tf", true);
        get_parameter("publish_tf", publish_tf_);
        declare_parameter("debug_flag", false);
        get_parameter("debug_flag", debug_flag_);

        declare_parameter("input_cloud_topic", "/full_pointcloud");
        get_parameter("input_cloud_topic", input_cloud_topic_);

        // Print params
        std::cout << "=== Localization Parameters ===" << std::endl;
        std::cout << "pcd_path: " << pcd_path_ << std::endl;
        std::cout << "registration_method: " << registration_method_ << std::endl;
        std::cout << "gicp_max_corr_dist: " << gicp_corr_dist << std::endl;
        std::cout << "gicp_max_iterations: " << gicp_max_iterations_ << std::endl;
        std::cout << "gicp_fitness_threshold: " << gicp_fitness_threshold_ << std::endl;
        std::cout << "voxel_leaf_size: " << voxel_leaf_size_ << std::endl;
        std::cout << "map_voxel_leaf_size: " << map_voxel_leaf_size_ << std::endl;
        std::cout << "loc_update_min_dist: " << loc_update_min_dist_ << std::endl;
        std::cout << "===============================" << std::endl;

        // Load map
        map_cloud_.reset(new PointCloudT());
        if (pcd_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "Parameter 'pcd_path' not set! Cannot start localization.");
            return;
        }

        if (pcl::io::loadPCDFile<PointT>(pcd_path_, *map_cloud_) == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to load map: %s", pcd_path_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Map loaded: %s (%zu points)", pcd_path_.c_str(), map_cloud_->size());

        // Downsample map
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(map_cloud_);
        vg.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
        vg.filter(*map_cloud_);
        RCLCPP_INFO(get_logger(), "Map after downsample: %zu points", map_cloud_->size());

        // Setup registration
        if (registration_method_ == "NDT") {
            auto ndt = std::make_shared<pcl::NormalDistributionsTransform<PointT, PointT>>();
            ndt->setResolution(ndt_resolution);
            ndt->setTransformationEpsilon(0.01);
            ndt->setMaximumIterations(gicp_max_iterations_);
            registration_ = ndt;
        } else {
            auto gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>();
            gicp->setMaxCorrespondenceDistance(gicp_corr_dist);
            gicp->setTransformationEpsilon(1e-8);
            gicp->setMaximumIterations(gicp_max_iterations_);
            gicp->setEuclideanFitnessEpsilon(1e-6);
            registration_ = gicp;
        }

        // Set map as target
        registration_->setInputTarget(map_cloud_);
        map_loaded_ = true;

        // Initialize pose
        current_pose_ = Eigen::Matrix4f::Identity();
        path_.header.frame_id = global_frame_id_;

        // Subscribers
        input_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::cloudCallback, this, std::placeholders::_1));

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&RgbdCamLoc::odomCallback, this, std::placeholders::_1));

        sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&RgbdCamLoc::initialPoseCallback, this, std::placeholders::_1));

        // Publishers
        pub_map_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "loc/map_cloud", rclcpp::QoS(10));
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "loc/pose", rclcpp::QoS(10));
        pub_path_ = create_publisher<nav_msgs::msg::Path>(
            "loc/trajectory", rclcpp::QoS(10));

        // Map publish timer
        map_pub_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&RgbdCamLoc::publishMapTimer, this));

        RCLCPP_INFO(get_logger(), "=== LOCALIZATION READY === Map: %s", pcd_path_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!map_loaded_ || msg->width * msg->height == 0 || !odom_received_) return;

        PointCloudT::Ptr cloud(new PointCloudT());
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // Transform to robot_frame if needed
        if (msg->header.frame_id != robot_frame_id_) {
            try {
                tf2::TimePoint time_point = tf2::TimePoint(
                    std::chrono::seconds(msg->header.stamp.sec) +
                    std::chrono::nanoseconds(msg->header.stamp.nanosec));
                const geometry_msgs::msg::TransformStamped transform =
                    tfbuffer_.lookupTransform(robot_frame_id_, msg->header.frame_id, time_point);
                Eigen::Affine3d affine = tf2::transformToEigen(transform);
                PointCloudT::Ptr transformed(new PointCloudT());
                pcl::transformPointCloud(*cloud, *transformed,
                                        affine.matrix().cast<float>());
                cloud = transformed;
            } catch (tf2::TransformException &) {
                return;
            }
        }

        // Range filter
        if (use_min_max_filter_) {
            PointCloudT::Ptr filtered(new PointCloudT());
            for (const auto & p : cloud->points) {
                double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                if (scan_min_range_ < r && r < scan_max_range_) {
                    filtered->points.push_back(p);
                }
            }
            cloud = filtered;
        }

        // Voxel filter
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        vg.filter(*cloud);

        if (cloud->size() < 50) return;

        // Transform to odom frame using current odometry
        Eigen::Affine3f odom_tf = Eigen::Affine3f::Identity();
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_tf.translation() << robot_x_, robot_y_, robot_z_;
            odom_tf.rotate(Eigen::Quaternionf(robot_qw_, robot_qx_, robot_qy_, robot_qz_));
        }

        PointCloudT::Ptr cloud_in_odom(new PointCloudT());
        pcl::transformPointCloud(*cloud, *cloud_in_odom, odom_tf);

        // Check minimal movement before updating localization
        Eigen::Vector3f current_pos(odom_tf.translation());
        if (loc_initialized_ && (current_pos - last_loc_pos_).norm() < loc_update_min_dist_) {
            publishTF(msg->header.stamp);
            return;
        }

        // Scan matching vs map
        registration_->setInputSource(cloud_in_odom);
        PointCloudT::Ptr aligned(new PointCloudT());
        registration_->align(*aligned, current_pose_);

        if (registration_->hasConverged() &&
            registration_->getFitnessScore() < gicp_fitness_threshold_) {
            current_pose_ = registration_->getFinalTransformation();
            loc_initialized_ = true;
            last_loc_pos_ = current_pos;

            if (debug_flag_) {
                RCLCPP_INFO(get_logger(), "Localization updated. Fitness: %.4f",
                           registration_->getFitnessScore());
            }
        }

        publishTF(msg->header.stamp);

        // Publish pose
        Eigen::Matrix4f pose_map = current_pose_ * odom_tf.matrix();
        Eigen::Vector3d position = pose_map.block<3, 1>(0, 3).cast<double>();
        Eigen::Matrix3d rot_mat = pose_map.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond quat_eig(rot_mat);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = msg->header.stamp;
        ps.header.frame_id = global_frame_id_;
        ps.pose.position.x = position.x();
        ps.pose.position.y = position.y();
        ps.pose.position.z = position.z();
        ps.pose.orientation = tf2::toMsg(quat_eig);
        pub_pose_->publish(ps);

        path_.poses.push_back(ps);
        pub_path_->publish(path_);
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
        RCLCPP_INFO(get_logger(), "Initial pose received!");
        Eigen::Affine3d init_affine;
        tf2::fromMsg(msg->pose.pose, init_affine);
        current_pose_ = init_affine.matrix().cast<float>();
        loc_initialized_ = true;
    }

    void publishTF(const rclcpp::Time & stamp) {
        if (!publish_tf_) return;

        Eigen::Affine3f aff(current_pose_);
        Eigen::Quaternionf q(aff.rotation());

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = global_frame_id_;
        t.child_frame_id = odom_frame_id_;
        t.transform.translation.x = aff.translation().x();
        t.transform.translation.y = aff.translation().y();
        t.transform.translation.z = aff.translation().z();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        broadcaster_.sendTransform(t);
    }

    void publishMapTimer() {
        if (!map_loaded_ || map_cloud_->empty()) return;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*map_cloud_, output);
        output.header.stamp = now();
        output.header.frame_id = global_frame_id_;
        pub_map_cloud_->publish(output);
    }

    // Parameters
    std::string pcd_path_;
    std::string global_frame_id_, robot_frame_id_, odom_frame_id_;
    std::string registration_method_, input_cloud_topic_;
    double voxel_leaf_size_, map_voxel_leaf_size_;
    int gicp_max_iterations_;
    double gicp_fitness_threshold_;
    bool use_min_max_filter_;
    double scan_min_range_, scan_max_range_;
    double loc_update_min_dist_;
    bool publish_tf_, debug_flag_;

    // TF2
    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    // Registration
    pcl::Registration<PointT, PointT>::Ptr registration_;

    // State
    bool map_loaded_ = false;
    bool odom_received_ = false;
    bool loc_initialized_ = false;
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f last_loc_pos_ = Eigen::Vector3f::Zero();

    float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
    float robot_qx_ = 0, robot_qy_ = 0, robot_qz_ = 0, robot_qw_ = 1;
    std::mutex odom_mutex_;

    PointCloudT::Ptr map_cloud_;
    nav_msgs::msg::Path path_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdCamLoc>());
    rclcpp::shutdown();
    return 0;
}
