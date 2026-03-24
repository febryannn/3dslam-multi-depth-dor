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

// G2O HEADERS UNTUK BACKEND (POSE GRAPH OPTIMIZATION + LOOP CLOSURE)
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

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
 *
 * Perbaikan utama:
 *   - Subscribe /odom topic langsung (bukan TF lookup) untuk initial guess
 *   - Publish odom→base_link TF sebagai fallback (fix broken TF tree)
 *   - Fitness score rejection untuk tolak scan matching buruk
 *   - Parameter GICP disesuaikan untuk RGBD (correspondence dist lebih kecil)
 *   - Rotation trigger untuk submap update
 */
class SlamRgbdCam : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    SlamRgbdCam() : Node("slam_rgbd_cam") {
        // --- Parameter ---
        this->declare_parameter("registration_method", "GICP");
        this->declare_parameter("ndt_resolution", 0.5);
        this->declare_parameter("gicp_corr_dist_threshold", 1.0);
        this->declare_parameter("trans_for_mapupdate", 0.3);
        this->declare_parameter("angle_for_mapupdate", 0.3);
        this->declare_parameter("vg_size_for_input", 0.05);
        this->declare_parameter("vg_size_for_map", 0.05);
        this->declare_parameter("num_targeted_cloud", 20);
        this->declare_parameter("map_publish_period", 5.0);
        this->declare_parameter("scan_min_range", 0.5);
        this->declare_parameter("scan_max_range", 4.0);
        this->declare_parameter("crop_min_z", -0.3);
        this->declare_parameter("crop_max_z", 2.5);
        this->declare_parameter("use_odom", true);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("global_frame_id", "map");
        this->declare_parameter("robot_frame_id", "base_link");
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("map_save_path", "/tmp/slam_map");
        this->declare_parameter("input_cloud_topic", "/full_pointcloud");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("fitness_score_threshold", 0.5);

        // Parameter Loop Closure + G2O Backend (ala lidarslam_ros2)
        this->declare_parameter("enable_loop_closure", true);
        this->declare_parameter("loop_detection_period", 1000);           // ms, timer period
        this->declare_parameter("threshold_loop_closure_score", 1.0);     // Fitness score max
        this->declare_parameter("distance_loop_closure", 20.0);           // Min traveled distance untuk LC
        this->declare_parameter("range_of_searching_loop_closure", 20.0); // Radius pencarian (m)
        this->declare_parameter("search_submap_num", 3);                  // Jumlah submap adjacent
        this->declare_parameter("num_adjacent_pose_cnstraints", 5);       // Edge constraint adjacent
        this->declare_parameter("g2o_iterations", 10);

        registration_method_ = this->get_parameter("registration_method").as_string();
        ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
        gicp_corr_dist_ = this->get_parameter("gicp_corr_dist_threshold").as_double();
        trans_for_mapupdate_ = this->get_parameter("trans_for_mapupdate").as_double();
        angle_for_mapupdate_ = this->get_parameter("angle_for_mapupdate").as_double();
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
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        fitness_score_threshold_ = this->get_parameter("fitness_score_threshold").as_double();

        // Loop Closure + G2O params
        enable_loop_closure_ = this->get_parameter("enable_loop_closure").as_bool();
        loop_detection_period_ = this->get_parameter("loop_detection_period").as_int();
        threshold_loop_closure_score_ = this->get_parameter("threshold_loop_closure_score").as_double();
        distance_loop_closure_ = this->get_parameter("distance_loop_closure").as_double();
        range_of_searching_loop_closure_ = this->get_parameter("range_of_searching_loop_closure").as_double();
        search_submap_num_ = this->get_parameter("search_submap_num").as_int();
        num_adjacent_pose_cnstraints_ = this->get_parameter("num_adjacent_pose_cnstraints").as_int();
        g2o_iterations_ = this->get_parameter("g2o_iterations").as_int();

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

        // Subscribe /odom langsung (bukan TF lookup) — lebih reliable
        if (use_odom_) {
            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, rclcpp::QoS(50),
                std::bind(&SlamRgbdCam::odomCallback, this, std::placeholders::_1));
        }

        pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/slam/map_cloud", rclcpp::QoS(1).durability_volatile());
        pub_current_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/slam/current_scan", rclcpp::QoS(1).durability_volatile());
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/slam/trajectory", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);

        // Modified map publishers (setelah g2o optimization)
        pub_modified_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/slam/modified_map", rclcpp::QoS(1).durability_volatile());
        pub_modified_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/slam/modified_path", 10);

        // Timer untuk loop closure detection (ala lidarslam_ros2)
        if (enable_loop_closure_) {
            std::chrono::milliseconds period(loop_detection_period_);
            loop_detect_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&SlamRgbdCam::searchLoop, this));
        }

        save_map_srv_ = this->create_service<std_srvs::srv::Empty>(
            "/slam/save_map",
            std::bind(&SlamRgbdCam::saveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        last_map_publish_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "=== 3D SLAM READY (scan-to-submap) ===");
        RCLCPP_INFO(this->get_logger(), "  Method: %s | Input: %s | Odom: %s",
                    registration_method_.c_str(), input_topic.c_str(), odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  GICP corr_dist: %.2f | Fitness threshold: %.2f",
                    gicp_corr_dist_, fitness_score_threshold_);
        RCLCPP_INFO(this->get_logger(), "  Submap trigger: %.2fm / %.2frad | Target submaps: %d",
                    trans_for_mapupdate_, angle_for_mapupdate_, num_targeted_cloud_);
        RCLCPP_INFO(this->get_logger(), "  Save: ros2 service call /slam/save_map std_srvs/srv/Empty");
        RCLCPP_INFO(this->get_logger(), "  Loop Closure: %s | dist_lc: %.1fm | range: %.1fm | search_submap: %d",
                    enable_loop_closure_ ? "ON" : "OFF", distance_loop_closure_,
                    range_of_searching_loop_closure_, search_submap_num_);
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
            gicp->setTransformationEpsilon(1e-6);
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

    // =============== Odom Callback (subscribe /odom langsung) ===============
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    // =============== Main Callback ===============
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->width * msg->height == 0) return;

        auto raw = std::make_shared<PointCloudT>();
        pcl::fromROSMsg(*msg, *raw);
        if (raw->empty()) return;

        auto cloud = filterCloud(raw);
        if (cloud->size() < 50) {
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
        RCLCPP_INFO(this->get_logger(), "Membuat map pertama (%zu pts)...", cloud->size());

        auto map_cloud = std::make_shared<PointCloudT>();
        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(vg_size_map_, vg_size_map_, vg_size_map_);
        vg.setInputCloud(cloud);
        vg.filter(*map_cloud);

        // Transform ke global (identity pada awal)
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

        // Initial guess dari odometry
        Eigen::Matrix4f initial_guess = current_pose_;
        if (use_odom_) {
            initial_guess = getOdomBasedGuess();
        }

        // Align
        auto aligned = std::make_shared<PointCloudT>();
        registration_->align(*aligned, initial_guess);

        bool match_accepted = false;
        if (registration_->hasConverged()) {
            double fitness = registration_->getFitnessScore();
            Eigen::Matrix4f result = registration_->getFinalTransformation();

            // Reject match jika fitness score terlalu tinggi (bad match)
            if (fitness < fitness_score_threshold_) {
                current_pose_ = result;
                match_accepted = true;
                RCLCPP_DEBUG(this->get_logger(), "Converged. Fitness: %.4f", fitness);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Fitness terlalu tinggi: %.4f > %.4f, pakai odom/prev pose",
                    fitness, fitness_score_threshold_);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Scan matching gagal converge");
        }

        // Fallback ke initial guess jika match ditolak
        if (!match_accepted) {
            current_pose_ = initial_guess;
        }

        // Cek apakah perlu submap baru (translasi ATAU rotasi)
        Eigen::Vector3f trans_diff = current_pose_.block<3, 1>(0, 3) - last_submap_pose_.block<3, 1>(0, 3);
        float dist = trans_diff.norm();

        // Hitung perbedaan rotasi
        Eigen::Matrix3f rot_diff = last_submap_pose_.block<3, 3>(0, 0).transpose() *
                                    current_pose_.block<3, 3>(0, 0);
        float angle = std::acos(std::min(1.0f, std::max(-1.0f,
            (rot_diff.trace() - 1.0f) / 2.0f)));

        if ((dist > trans_for_mapupdate_ || angle > angle_for_mapupdate_) && !mapping_flag_) {
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

    // =============== Odometry Initial Guess (dari /odom topic) ===============
    Eigen::Matrix4f getOdomBasedGuess() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!has_odom_) return current_pose_;

        // Konversi odom message ke matrix
        const auto& pos = latest_odom_.pose.pose.position;
        const auto& ori = latest_odom_.pose.pose.orientation;

        Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
        Eigen::Matrix4f odom_mat = Eigen::Matrix4f::Identity();
        odom_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        odom_mat(0, 3) = pos.x;
        odom_mat(1, 3) = pos.y;
        odom_mat(2, 3) = pos.z;

        if (has_previous_odom_) {
            Eigen::Matrix4f delta = previous_odom_mat_.inverse() * odom_mat;
            previous_odom_mat_ = odom_mat;
            return current_pose_ * delta;
        }

        previous_odom_mat_ = odom_mat;
        has_previous_odom_ = true;
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
        is_map_array_updated_ = true;  // Signal ke loop detection timer

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
        // Selalu publish odom→base_link dari /odom data
        // (fallback karena Gazebo bridge sering gagal forward TF)
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (has_odom_) {
                const auto& pos = latest_odom_.pose.pose.position;
                const auto& ori = latest_odom_.pose.pose.orientation;

                geometry_msgs::msg::TransformStamped odom_tf;
                odom_tf.header.stamp = stamp;
                odom_tf.header.frame_id = odom_frame_id_;
                odom_tf.child_frame_id = robot_frame_id_;
                odom_tf.transform.translation.x = pos.x;
                odom_tf.transform.translation.y = pos.y;
                odom_tf.transform.translation.z = pos.z;
                odom_tf.transform.rotation = ori;
                tf_broadcaster_->sendTransform(odom_tf);
            }
        }

        if (use_odom_) {
            // Compute map→odom dari stored odom data
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (has_odom_) {
                const auto& pos = latest_odom_.pose.pose.position;
                const auto& ori = latest_odom_.pose.pose.orientation;

                Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
                Eigen::Matrix4f odom_T_base = Eigen::Matrix4f::Identity();
                odom_T_base.block<3, 3>(0, 0) = q.toRotationMatrix();
                odom_T_base(0, 3) = pos.x;
                odom_T_base(1, 3) = pos.y;
                odom_T_base(2, 3) = pos.z;

                // map_T_odom = map_T_base * (odom_T_base)^-1
                Eigen::Matrix4f map_T_odom = current_pose_ * odom_T_base.inverse();

                Eigen::Affine3f affine(map_T_odom);
                Eigen::Quaternionf q_out(affine.rotation());

                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = stamp;
                tf_msg.header.frame_id = global_frame_id_;
                tf_msg.child_frame_id = odom_frame_id_;
                tf_msg.transform.translation.x = affine.translation().x();
                tf_msg.transform.translation.y = affine.translation().y();
                tf_msg.transform.translation.z = affine.translation().z();
                tf_msg.transform.rotation.x = q_out.x();
                tf_msg.transform.rotation.y = q_out.y();
                tf_msg.transform.rotation.z = q_out.z();
                tf_msg.transform.rotation.w = q_out.w();

                tf_broadcaster_->sendTransform(tf_msg);
            } else {
                // Belum ada odom, publish map→base_link langsung
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

    // =============== Loop Closure Detection (ala lidarslam_ros2) ===============
    struct LoopEdge {
        std::pair<int, int> pair_id;
        Eigen::Isometry3d relative_pose;
    };

    void searchLoop() {
        if (submaps_.size() < 3) return;
        if (!is_map_array_updated_) return;
        is_map_array_updated_ = false;

        std::lock_guard<std::mutex> lock(map_mutex_);
        int num_submaps = static_cast<int>(submaps_.size());

        // Latest submap
        auto& latest_submap = submaps_[num_submaps - 1];
        double latest_moving_distance = latest_submap.distance;
        Eigen::Vector3f latest_pos = latest_submap.pose.block<3, 1>(0, 3);

        // Transform latest submap cloud ke global frame (source untuk matching)
        auto latest_cloud_global = std::make_shared<PointCloudT>();
        pcl::transformPointCloud(*latest_submap.cloud, *latest_cloud_global, latest_submap.pose);
        registration_->setInputSource(latest_cloud_global);

        // Cari kandidat loop closure:
        // - Jarak tempuh (traveled distance) harus > distance_loop_closure_
        // - Jarak Euclidean harus < range_of_searching_loop_closure_
        bool is_candidate = false;
        int id_min = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < num_submaps; i++) {
            Eigen::Vector3f submap_pos = submaps_[i].pose.block<3, 1>(0, 3);
            double euclidean_dist = (latest_pos - submap_pos).cast<double>().norm();

            if (latest_moving_distance - submaps_[i].distance > distance_loop_closure_ &&
                euclidean_dist < range_of_searching_loop_closure_) {
                is_candidate = true;
                if (euclidean_dist < min_dist) {
                    id_min = i;
                    min_dist = euclidean_dist;
                }
            }
        }

        if (!is_candidate) return;

        // Bangun target cloud dari multiple adjacent submaps (seperti lidarslam_ros2)
        auto submap_clouds = std::make_shared<PointCloudT>();
        for (int j = 0; j <= 2 * search_submap_num_; ++j) {
            int idx = id_min + j - search_submap_num_;
            if (idx < 0 || idx >= num_submaps) continue;
            auto tmp = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*submaps_[idx].cloud, *tmp, submaps_[idx].pose);
            *submap_clouds += *tmp;
        }

        // Voxel filter target cloud
        auto filtered_target = std::make_shared<PointCloudT>();
        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(vg_size_map_, vg_size_map_, vg_size_map_);
        vg.setInputCloud(submap_clouds);
        vg.filter(*filtered_target);

        registration_->setInputTarget(filtered_target);

        // Align
        auto output_cloud = std::make_shared<PointCloudT>();
        registration_->align(*output_cloud);
        double fitness_score = registration_->getFitnessScore();

        if (fitness_score < threshold_loop_closure_score_) {
            // Loop closure terdeteksi!
            // Hitung relative pose (sama persis dengan lidarslam_ros2)
            Eigen::Isometry3d from_pose(submaps_[id_min].pose.cast<double>());
            Eigen::Isometry3d to_pose(
                (registration_->getFinalTransformation().cast<double>()) *
                Eigen::Isometry3d(latest_submap.pose.cast<double>()).matrix());

            LoopEdge loop_edge;
            loop_edge.pair_id = std::pair<int, int>(id_min, num_submaps - 1);
            loop_edge.relative_pose = Eigen::Isometry3d(from_pose.inverse() * to_pose);
            loop_edges_.push_back(loop_edge);

            RCLCPP_INFO(this->get_logger(),
                "== LOOP CLOSURE DETECTED == id:%d -> %d, dist:%.2fm, score:%.4f",
                id_min, num_submaps - 1, submaps_[id_min].distance, fitness_score);

            doPoseAdjustment();
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                "Loop candidate %d rejected (fitness:%.4f > threshold:%.4f)",
                id_min, fitness_score, threshold_loop_closure_score_);
        }
    }

    // =============== Pose Graph Optimization dengan G2O ===============
    void doPoseAdjustment() {
        // map_mutex_ sudah di-lock oleh searchLoop()

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
            std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));
        optimizer.setAlgorithm(solver);

        int submaps_size = static_cast<int>(submaps_.size());
        Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();

        // 1. Tambah Vertex + Multiple Adjacent Pose Constraints
        for (int i = 0; i < submaps_size; i++) {
            Eigen::Isometry3d pose(submaps_[i].pose.cast<double>());

            g2o::VertexSE3* vertex_se3 = new g2o::VertexSE3();
            vertex_se3->setId(i);
            vertex_se3->setEstimate(pose);
            if (i == 0) vertex_se3->setFixed(true);
            optimizer.addVertex(vertex_se3);

            // Multiple adjacent pose constraints (bukan hanya i-1 → i)
            if (i > num_adjacent_pose_cnstraints_) {
                for (int j = 0; j < num_adjacent_pose_cnstraints_; j++) {
                    int pre_idx = i - num_adjacent_pose_cnstraints_ + j;
                    Eigen::Isometry3d pre_pose(submaps_[pre_idx].pose.cast<double>());
                    Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;

                    g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
                    edge_se3->setMeasurement(relative_pose);
                    edge_se3->setInformation(info_mat);
                    edge_se3->vertices()[0] = optimizer.vertex(pre_idx);
                    edge_se3->vertices()[1] = optimizer.vertex(i);
                    optimizer.addEdge(edge_se3);
                }
            } else if (i > 0) {
                // Untuk node awal, minimal 1 edge sequential
                Eigen::Isometry3d pre_pose(submaps_[i - 1].pose.cast<double>());
                Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;

                g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
                edge_se3->setMeasurement(relative_pose);
                edge_se3->setInformation(info_mat);
                edge_se3->vertices()[0] = optimizer.vertex(i - 1);
                edge_se3->vertices()[1] = optimizer.vertex(i);
                optimizer.addEdge(edge_se3);
            }
        }

        // 2. Tambah Loop Closure Edges
        for (const auto& lc : loop_edges_) {
            g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
            edge_se3->setMeasurement(lc.relative_pose);
            edge_se3->setInformation(info_mat);
            edge_se3->vertices()[0] = optimizer.vertex(lc.pair_id.first);
            edge_se3->vertices()[1] = optimizer.vertex(lc.pair_id.second);
            optimizer.addEdge(edge_se3);
        }

        RCLCPP_INFO(this->get_logger(), "G2O: Optimizing %d vertices, %zu LC edges...",
                    submaps_size, loop_edges_.size());

        optimizer.initializeOptimization();
        optimizer.optimize(g2o_iterations_);

        // 3. Update semua pose submap + publish modified map
        auto modified_map = std::make_shared<PointCloudT>();
        nav_msgs::msg::Path modified_path;
        modified_path.header.frame_id = global_frame_id_;

        for (int i = 0; i < submaps_size; i++) {
            g2o::VertexSE3* v = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
            Eigen::Matrix4f optimized_pose = v->estimate().matrix().cast<float>();
            submaps_[i].pose = optimized_pose;

            // Transform cloud dengan pose baru
            auto tmp = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*submaps_[i].cloud, *tmp, optimized_pose);
            *modified_map += *tmp;

            // Path
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = global_frame_id_;
            Eigen::Quaternionf q(Eigen::Matrix3f(optimized_pose.block<3, 3>(0, 0)));
            ps.pose.position.x = optimized_pose(0, 3);
            ps.pose.position.y = optimized_pose(1, 3);
            ps.pose.position.z = optimized_pose(2, 3);
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            ps.pose.orientation.w = q.w();
            modified_path.poses.push_back(ps);
        }

        // 4. Update current pose + state
        current_pose_ = submaps_.back().pose;
        last_submap_pose_ = current_pose_;
        path_msg_ = modified_path;

        // 5. Rebuild targeted cloud
        PointCloudT new_targeted;
        int n = static_cast<int>(submaps_.size());
        for (int i = std::max(0, n - num_targeted_cloud_); i < n; i++) {
            auto tmp = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*submaps_[i].cloud, *tmp, submaps_[i].pose);
            new_targeted += *tmp;
        }
        targeted_cloud_copy_ = new_targeted;
        is_map_updated_ = true;

        // 6. Publish modified map + path
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*modified_map, map_msg);
        map_msg.header.frame_id = global_frame_id_;
        map_msg.header.stamp = this->now();
        pub_modified_map_->publish(map_msg);
        pub_modified_path_->publish(modified_path);

        RCLCPP_INFO(this->get_logger(), "G2O: Optimization done. Drift corrected. Published modified map.");
    }

    // =============== Data ===============
    struct SubMap {
        PointCloudT::Ptr cloud;
        Eigen::Matrix4f pose;
        double distance;
    };

    // Params
    std::string registration_method_, global_frame_id_, robot_frame_id_, odom_frame_id_, map_save_path_;
    double ndt_resolution_, gicp_corr_dist_, trans_for_mapupdate_, angle_for_mapupdate_;
    double vg_size_input_, vg_size_map_, map_publish_period_;
    double scan_min_range_, scan_max_range_, crop_min_z_, crop_max_z_;
    double fitness_score_threshold_;
    int num_targeted_cloud_;
    bool use_odom_, publish_tf_;

    // Loop Closure + G2O params (ala lidarslam_ros2)
    bool enable_loop_closure_;
    int loop_detection_period_;
    double threshold_loop_closure_score_;
    double distance_loop_closure_;
    double range_of_searching_loop_closure_;
    int search_submap_num_;
    int num_adjacent_pose_cnstraints_;
    int g2o_iterations_;

    // Registration
    pcl::Registration<PointT, PointT>::Ptr registration_;

    // State
    bool is_first_frame_ = true;
    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f previous_odom_mat_;
    bool has_previous_odom_ = false;
    Eigen::Matrix4f last_submap_pose_ = Eigen::Matrix4f::Identity();

    // Odometry (subscribe /odom topic langsung)
    nav_msgs::msg::Odometry latest_odom_;
    std::mutex odom_mutex_;
    bool has_odom_ = false;

    // Submaps + Loop Closure
    std::vector<SubMap> submaps_;
    std::vector<LoopEdge> loop_edges_;
    PointCloudT targeted_cloud_copy_;
    std::mutex map_mutex_;
    std::atomic<bool> is_map_array_updated_{false};

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_, pub_current_scan_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_modified_map_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_modified_path_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
    rclcpp::TimerBase::SharedPtr loop_detect_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamRgbdCam>());
    rclcpp::shutdown();
    return 0;
}
