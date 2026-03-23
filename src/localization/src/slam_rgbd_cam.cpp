#include <memory>
#include <chrono>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include <future>
#include <fstream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

/**
 * 3D SLAM Node — Arsitektur scan-to-submap ala lidarslam_ros2
 *
 * Pipeline:
 *   1. pointcloud_concatenate menggabungkan 4 kamera (via TF2) → /full_pointcloud
 *   2. Node ini subscribe /full_pointcloud (sudah di frame base_link)
 *   3. Filter + downsample
 *   4. GICP/NDT scan-to-submap matching
 *   5. Update submap saat robot bergerak cukup jauh
 *   6. Publish map→odom TF, map cloud, trajectory
 */
class SlamRgbdCam : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    SlamRgbdCam() : Node("slam_rgbd_cam") {
        // --- Parameter ---
        this->declare_parameter("registration_method", "GICP");
        this->declare_parameter("ndt_resolution", 1.0);
        this->declare_parameter("gicp_corr_dist_threshold", 5.0);
        this->declare_parameter("trans_for_mapupdate", 0.5);
        this->declare_parameter("vg_size_for_input", 0.1);
        this->declare_parameter("vg_size_for_map", 0.05);
        this->declare_parameter("num_targeted_cloud", 10);
        this->declare_parameter("map_publish_period", 5.0);
        this->declare_parameter("scan_min_range", 0.3);
        this->declare_parameter("scan_max_range", 8.0);
        this->declare_parameter("crop_min_z", -0.3);
        this->declare_parameter("crop_max_z", 2.5);
        this->declare_parameter("use_odom", true);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("global_frame_id", "map");
        this->declare_parameter("robot_frame_id", "base_link");
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("map_save_path", "/tmp/slam_map");
        this->declare_parameter("input_cloud_topic", "/full_pointcloud");

        registration_method_ = this->get_parameter("registration_method").as_string();
        ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
        gicp_corr_dist_ = this->get_parameter("gicp_corr_dist_threshold").as_double();
        trans_for_mapupdate_ = this->get_parameter("trans_for_mapupdate").as_double();
        vg_size_input_ = this->get_parameter("vg_size_for_input").as_double();
        vg_size_map_ = this->get_parameter("vg_size_for_map").as_double();
        num_targeted_cloud_ = this->get_parameter("num_targeted_cloud").as_int();
        map_publish_period_ = this->get_parameter("map_publish_period").as_double();
        scan_min_range_ = this->get_parameter("scan_min_range").as_double();
        scan_max_range_ = this->get_parameter("scan_max_range").as_double();
        crop_min_z_ = this->get_parameter("crop_min_z").as_double();
        crop_max_z_ = this->get_parameter("crop_max_z").as_double();
        use_odom_ = this->get_parameter("use_odom").as_bool();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        global_frame_id_ = this->get_parameter("global_frame_id").as_string();
        robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        map_save_path_ = this->get_parameter("map_save_path").as_string();
        std::string input_topic = this->get_parameter("input_cloud_topic").as_string();

        // --- Registration algorithm ---
        setupRegistration();

        // --- Init ---
        path_msg_.header.frame_id = global_frame_id_;
        current_pose_ = Eigen::Matrix4f::Identity();
        previous_odom_mat_ = Eigen::Matrix4f::Identity();

        // --- TF ---
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- Sub/Pub ---
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&SlamRgbdCam::cloudCallback, this, std::placeholders::_1));

        pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/slam/map_cloud", rclcpp::QoS(1).durability_volatile());
        pub_current_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/slam/current_scan", rclcpp::QoS(1).durability_volatile());
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/slam/trajectory", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);

        save_map_srv_ = this->create_service<std_srvs::srv::Empty>(
            "/slam/save_map",
            std::bind(&SlamRgbdCam::saveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        last_map_publish_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "=== 3D SLAM READY (scan-to-submap) ===");
        RCLCPP_INFO(this->get_logger(), "  Method: %s | Input: %s",
                    registration_method_.c_str(), input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Submap trigger: %.2fm | Target submaps: %d",
                    trans_for_mapupdate_, num_targeted_cloud_);
        RCLCPP_INFO(this->get_logger(), "  Save: ros2 service call /slam/save_map std_srvs/srv/Empty");
    }

    ~SlamRgbdCam() { saveMap(); }

private:
    // =============== Registration Setup ===============
    void setupRegistration() {
        if (registration_method_ == "NDT") {
            auto ndt = std::make_shared<pcl::NormalDistributionsTransform<PointT, PointT>>();
            ndt->setResolution(ndt_resolution_);
            ndt->setTransformationEpsilon(0.01);
            ndt->setMaximumIterations(50);
            registration_ = ndt;
            RCLCPP_INFO(this->get_logger(), "NDT (resolution=%.2f)", ndt_resolution_);
        } else {
            auto gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>();
            gicp->setMaxCorrespondenceDistance(gicp_corr_dist_);
            gicp->setTransformationEpsilon(1e-8);
            gicp->setMaximumIterations(64);
            gicp->setEuclideanFitnessEpsilon(1e-6);
            registration_ = gicp;
            RCLCPP_INFO(this->get_logger(), "GICP (corr_dist=%.2f)", gicp_corr_dist_);
        }
    }

    // =============== Filtering ===============
    PointCloudT::Ptr filterCloud(const PointCloudT::Ptr& input) {
        auto filtered = std::make_shared<PointCloudT>();

        // Range + Z filter
        for (const auto& pt : input->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range < scan_min_range_ || range > scan_max_range_) continue;
            if (pt.z < crop_min_z_ || pt.z > crop_max_z_) continue;
            filtered->push_back(pt);
        }
        if (filtered->empty()) return filtered;

        // Voxel grid
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(filtered);
        vg.setLeafSize(vg_size_input_, vg_size_input_, vg_size_input_);
        vg.filter(*filtered);

        return filtered;
    }

    // =============== Main Callback ===============
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->width * msg->height == 0) return;

        auto raw = std::make_shared<PointCloudT>();
        pcl::fromROSMsg(*msg, *raw);
        if (raw->empty()) return;

        auto cloud = filterCloud(raw);
        if (cloud->size() < 30) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Cloud terlalu sedikit: %zu", cloud->size());
            return;
        }

        rclcpp::Time stamp = msg->header.stamp;

        if (is_first_frame_) {
            initializeMap(cloud, stamp);
            is_first_frame_ = false;
            return;
        }

        receiveCloud(cloud, stamp);
    }

    // =============== Initialize Map ===============
    void initializeMap(const PointCloudT::Ptr& cloud, const rclcpp::Time& stamp) {
        RCLCPP_INFO(this->get_logger(), "Membuat map pertama...");

        auto map_cloud = std::make_shared<PointCloudT>();
        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(vg_size_map_, vg_size_map_, vg_size_map_);
        vg.setInputCloud(cloud);
        vg.filter(*map_cloud);

        // Transform ke global (identity)
        auto transformed = std::make_shared<PointCloudT>();
        pcl::transformPointCloud(*map_cloud, *transformed, current_pose_);

        registration_->setInputTarget(transformed);

        SubMap submap;
        submap.cloud = map_cloud;
        submap.pose = current_pose_;
        submap.distance = 0.0;
        submaps_.push_back(submap);

        publishPose(current_pose_, stamp);
        publishMapCloud(stamp);
        if (publish_tf_) publishTF(stamp);

        RCLCPP_INFO(this->get_logger(), "Map pertama OK! (%zu pts). SLAM aktif.", map_cloud->size());
    }

    // =============== Scan-to-Submap Matching ===============
    void receiveCloud(const PointCloudT::Ptr& cloud, const rclcpp::Time& stamp) {
        // Cek background map update selesai
        if (mapping_flag_ && mapping_future_.valid()) {
            auto status = mapping_future_.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready) {
                if (is_map_updated_) {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    auto target = std::make_shared<PointCloudT>(targeted_cloud_copy_);
                    if (registration_method_ != "NDT") {
                        pcl::VoxelGrid<PointT> vg;
                        vg.setLeafSize(vg_size_input_, vg_size_input_, vg_size_input_);
                        vg.setInputCloud(target);
                        vg.filter(*target);
                    }
                    registration_->setInputTarget(target);
                    is_map_updated_ = false;
                }
                mapping_flag_ = false;
            }
        }

        // Set source
        registration_->setInputSource(cloud);

        // Initial guess
        Eigen::Matrix4f initial_guess = current_pose_;
        if (use_odom_) {
            initial_guess = getOdomBasedGuess(stamp);
        }

        // Align
        auto aligned = std::make_shared<PointCloudT>();
        registration_->align(*aligned, initial_guess);

        if (registration_->hasConverged()) {
            current_pose_ = registration_->getFinalTransformation();
            double fitness = registration_->getFitnessScore();
            RCLCPP_DEBUG(this->get_logger(), "Converged. Fitness: %.4f", fitness);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Scan matching gagal converge, pakai odometry");
            if (use_odom_) current_pose_ = initial_guess;
        }

        // Cek apakah perlu submap baru
        Eigen::Vector3f diff = current_pose_.block<3, 1>(0, 3) - last_submap_pose_.block<3, 1>(0, 3);
        float trans_diff = diff.norm();

        if (trans_diff > trans_for_mapupdate_ && !mapping_flag_) {
            last_submap_pose_ = current_pose_;
            mapping_flag_ = true;
            auto cloud_copy = std::make_shared<PointCloudT>(*cloud);
            Eigen::Matrix4f pose_copy = current_pose_;
            mapping_future_ = std::async(std::launch::async,
                &SlamRgbdCam::updateMap, this, cloud_copy, pose_copy);
        }

        // Publish
        publishPose(current_pose_, stamp);
        publishCurrentScan(cloud, current_pose_, stamp);
        if (publish_tf_) publishTF(stamp);

        double dt = (this->now() - last_map_publish_time_).seconds();
        if (dt > map_publish_period_) {
            publishMapCloud(stamp);
            last_map_publish_time_ = this->now();
        }

        pub_path_->publish(path_msg_);
    }

    // =============== Odometry Initial Guess ===============
    Eigen::Matrix4f getOdomBasedGuess(const rclcpp::Time& /*stamp*/) {
        try {
            auto odom_tf = tf_buffer_->lookupTransform(
                odom_frame_id_, robot_frame_id_, tf2::TimePointZero);
            Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_tf);
            Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();

            if (has_previous_odom_) {
                Eigen::Matrix4f delta = previous_odom_mat_.inverse() * odom_mat;
                previous_odom_mat_ = odom_mat;
                return current_pose_ * delta;
            }
            previous_odom_mat_ = odom_mat;
            has_previous_odom_ = true;
        } catch (tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Odom TF gagal: %s", e.what());
        }
        return current_pose_;
    }

    // =============== Update Map (Background) ===============
    void updateMap(const PointCloudT::Ptr cloud_ptr, const Eigen::Matrix4f pose) {
        // Downsample
        auto filtered = std::make_shared<PointCloudT>();
        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(vg_size_map_, vg_size_map_, vg_size_map_);
        vg.setInputCloud(cloud_ptr);
        vg.filter(*filtered);

        // Transform ke global
        auto transformed = std::make_shared<PointCloudT>();
        pcl::transformPointCloud(*filtered, *transformed, pose);

        // Bangun targeted_cloud dari N submap terakhir + baru
        PointCloudT new_targeted;
        new_targeted += *transformed;

        int n = submaps_.size();
        for (int i = 0; i < num_targeted_cloud_ - 1; ++i) {
            int idx = n - 1 - i;
            if (idx < 0) continue;
            auto tmp = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*submaps_[idx].cloud, *tmp, submaps_[idx].pose);
            new_targeted += *tmp;
        }

        // Simpan submap
        SubMap submap;
        submap.cloud = filtered;
        submap.pose = pose;
        submap.distance = 0.0;
        if (!submaps_.empty()) {
            Eigen::Vector3f prev = submaps_.back().pose.block<3, 1>(0, 3);
            Eigen::Vector3f curr = pose.block<3, 1>(0, 3);
            submap.distance = submaps_.back().distance + (curr - prev).norm();
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            submaps_.push_back(submap);
            targeted_cloud_copy_ = new_targeted;
        }

        is_map_updated_ = true;

        RCLCPP_INFO(this->get_logger(), "Submap #%zu (dist: %.2fm, target: %zu pts)",
                    submaps_.size(), submap.distance, new_targeted.size());
    }

    // =============== Publish Functions ===============
    void publishPose(const Eigen::Matrix4f& pose, const rclcpp::Time& stamp) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = global_frame_id_;
        ps.pose.position.x = pose(0, 3);
        ps.pose.position.y = pose(1, 3);
        ps.pose.position.z = pose(2, 3);
        Eigen::Quaternionf q(Eigen::Matrix3f(pose.block<3, 3>(0, 0)));
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path_msg_.poses.push_back(ps);
        pub_pose_->publish(ps);
    }

    void publishCurrentScan(const PointCloudT::Ptr& cloud,
                            const Eigen::Matrix4f& pose,
                            const rclcpp::Time& stamp) {
        auto out = std::make_shared<PointCloudT>();
        pcl::transformPointCloud(*cloud, *out, pose);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*out, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = global_frame_id_;
        pub_current_scan_->publish(msg);
    }

    void publishMapCloud(const rclcpp::Time& stamp) {
        auto full = std::make_shared<PointCloudT>();
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            for (const auto& sm : submaps_) {
                auto tmp = std::make_shared<PointCloudT>();
                pcl::transformPointCloud(*sm.cloud, *tmp, sm.pose);
                *full += *tmp;
            }
        }
        if (full->empty()) return;

        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(vg_size_map_, vg_size_map_, vg_size_map_);
        vg.setInputCloud(full);
        vg.filter(*full);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*full, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = global_frame_id_;
        pub_map_cloud_->publish(msg);
    }

    void publishTF(const rclcpp::Time& stamp) {
        if (use_odom_) {
            try {
                // lookupTransform(target=odom, source=base) → odom_T_base
                auto odom_base_tf = tf_buffer_->lookupTransform(
                    odom_frame_id_, robot_frame_id_, tf2::TimePointZero);
                Eigen::Affine3d odom_base = tf2::transformToEigen(odom_base_tf);
                Eigen::Matrix4f odom_T_base = odom_base.matrix().cast<float>();

                // map_T_odom = map_T_base * (odom_T_base)^-1
                //            = current_pose_ * inv(odom_T_base)
                Eigen::Matrix4f map_T_odom = current_pose_ * odom_T_base.inverse();

                Eigen::Affine3f affine(map_T_odom);
                Eigen::Quaternionf q(affine.rotation());

                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = stamp;
                tf_msg.header.frame_id = global_frame_id_;
                tf_msg.child_frame_id = odom_frame_id_;
                tf_msg.transform.translation.x = affine.translation().x();
                tf_msg.transform.translation.y = affine.translation().y();
                tf_msg.transform.translation.z = affine.translation().z();
                tf_msg.transform.rotation.x = q.x();
                tf_msg.transform.rotation.y = q.y();
                tf_msg.transform.rotation.z = q.z();
                tf_msg.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(tf_msg);
            } catch (tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "map→odom TF error: %s", e.what());
            }
        } else {
            Eigen::Affine3f affine(current_pose_);
            Eigen::Quaternionf q(affine.rotation());

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = global_frame_id_;
            tf_msg.child_frame_id = robot_frame_id_;
            tf_msg.transform.translation.x = affine.translation().x();
            tf_msg.transform.translation.y = affine.translation().y();
            tf_msg.transform.translation.z = affine.translation().z();
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    // =============== Save Map ===============
    void saveMap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (submaps_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map kosong.");
            return;
        }
        auto full = std::make_shared<PointCloudT>();
        for (const auto& sm : submaps_) {
            auto tmp = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*sm.cloud, *tmp, sm.pose);
            *full += *tmp;
        }
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(full);
        vg.setLeafSize(vg_size_map_, vg_size_map_, vg_size_map_);
        vg.filter(*full);

        std::string pcd_path = map_save_path_ + ".pcd";
        pcl::io::savePCDFileBinaryCompressed(pcd_path, *full);
        RCLCPP_INFO(this->get_logger(), "Map saved: %s (%zu pts)", pcd_path.c_str(), full->size());

        std::string traj_path = map_save_path_ + "_trajectory.txt";
        std::ofstream f(traj_path);
        for (const auto& sm : submaps_) {
            const auto& p = sm.pose;
            f << p(0,3) << " " << p(1,3) << " " << p(2,3) << " "
              << p(0,0) << " " << p(0,1) << " " << p(0,2) << " "
              << p(1,0) << " " << p(1,1) << " " << p(1,2) << " "
              << p(2,0) << " " << p(2,1) << " " << p(2,2) << "\n";
        }
        f.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory saved: %s (%zu poses)", traj_path.c_str(), submaps_.size());
    }

    void saveMapCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>) {
        RCLCPP_INFO(this->get_logger(), "Save map requested...");
        saveMap();
    }

    // =============== Data ===============
    struct SubMap {
        PointCloudT::Ptr cloud;
        Eigen::Matrix4f pose;
        double distance;
    };

    // Params
    std::string registration_method_, global_frame_id_, robot_frame_id_, odom_frame_id_, map_save_path_;
    double ndt_resolution_, gicp_corr_dist_, trans_for_mapupdate_;
    double vg_size_input_, vg_size_map_, map_publish_period_;
    double scan_min_range_, scan_max_range_, crop_min_z_, crop_max_z_;
    int num_targeted_cloud_;
    bool use_odom_, publish_tf_;

    // Registration
    pcl::Registration<PointT, PointT>::Ptr registration_;

    // State
    bool is_first_frame_ = true;
    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f previous_odom_mat_;
    bool has_previous_odom_ = false;
    Eigen::Matrix4f last_submap_pose_ = Eigen::Matrix4f::Identity();

    // Submaps
    std::vector<SubMap> submaps_;
    PointCloudT targeted_cloud_copy_;
    std::mutex map_mutex_;

    // Background mapping
    std::atomic<bool> mapping_flag_{false};
    std::atomic<bool> is_map_updated_{false};
    std::future<void> mapping_future_;

    // Path
    nav_msgs::msg::Path path_msg_;
    rclcpp::Time last_map_publish_time_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_, pub_current_scan_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamRgbdCam>());
    rclcpp::shutdown();
    return 0;
}
