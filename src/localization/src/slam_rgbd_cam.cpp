#include <memory>
#include <chrono>
#include <iostream>
#include <cmath>
#include <mutex>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"

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
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <octomap/octomap.h>

#include <Eigen/Dense>

/**
 * 3D SLAM Node dengan Multi-Depth Camera (4 Kamera RGBD)
 *
 * Fitur:
 *  - GICP / NDT scan matching untuk registrasi frame-to-frame
 *  - Keyframe management (hanya registrasi saat robot bergerak cukup)
 *  - Local map sliding window untuk efisiensi GICP matching
 *  - OctoMap raycasting untuk 3D occupancy mapping
 *  - Map saving/loading (PCD + OctoMap binary)
 *  - Pose path trajectory publishing
 *  - Service untuk save map on-demand
 *  - Configurable via ROS2 parameters
 */
class SlamRgbdCam : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    SlamRgbdCam() : Node("slam_rgbd_cam") {
        // --- Deklarasi semua parameter ---
        this->declare_parameter("max_cam_d", 8.0);
        this->declare_parameter("min_cam_d", 0.5);
        this->declare_parameter("voxel_leaf_size", 0.06);
        this->declare_parameter("map_voxel_leaf_size", 0.08);
        this->declare_parameter("octomap_resolution", 0.1);
        this->declare_parameter("gicp_max_corr_dist", 1.0);
        this->declare_parameter("gicp_max_iterations", 50);
        this->declare_parameter("gicp_transform_epsilon", 1e-6);
        this->declare_parameter("gicp_fitness_threshold", 0.5);
        this->declare_parameter("keyframe_dist_threshold", 0.3);
        this->declare_parameter("keyframe_angle_threshold", 0.2);
        this->declare_parameter("local_map_size", 20);
        this->declare_parameter("crop_min_z", -0.5);
        this->declare_parameter("crop_max_z", 3.0);
        this->declare_parameter("self_filter_size", 0.6);
        this->declare_parameter("sor_mean_k", 30);
        this->declare_parameter("sor_stddev", 1.0);
        this->declare_parameter("use_ndt", false);
        this->declare_parameter("ndt_step_size", 0.1);
        this->declare_parameter("ndt_resolution", 1.0);
        this->declare_parameter("map_save_path", "/tmp/slam_map");
        this->declare_parameter("publish_octomap_cloud", true);
        this->declare_parameter("sensor_height", 0.4);

        // Posisi kamera relatif terhadap base_link
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

        // --- Ambil parameter ---
        loadParameters();

        // --- Inisialisasi data structure ---
        global_map_.reset(new PointCloudT());
        local_map_.reset(new PointCloudT());
        octo_map_ = std::make_shared<octomap::OcTree>(octomap_resolution_);
        octo_map_->setClampingThresMax(0.97);
        octo_map_->setClampingThresMin(0.12);
        octo_map_->setProbHit(0.7);
        octo_map_->setProbMiss(0.4);
        octo_map_->setOccupancyThres(0.5);

        c2_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        c3_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        c4_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        path_msg_.header.frame_id = "map";

        // --- Subscriptions ---
        sub_c1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in1/points", rclcpp::SensorDataQoS(),
            std::bind(&SlamRgbdCam::cam1MainCallback, this, std::placeholders::_1));
        sub_c2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in2/points", rclcpp::SensorDataQoS(),
            std::bind(&SlamRgbdCam::cam2Callback, this, std::placeholders::_1));
        sub_c3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in3/points", rclcpp::SensorDataQoS(),
            std::bind(&SlamRgbdCam::cam3Callback, this, std::placeholders::_1));
        sub_c4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in4/points", rclcpp::SensorDataQoS(),
            std::bind(&SlamRgbdCam::cam4Callback, this, std::placeholders::_1));

        sub_robot_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&SlamRgbdCam::robotOdomCallback, this, std::placeholders::_1));

        // --- Publishers ---
        pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map_cloud", 10);
        pub_current_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/current_scan", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/slam/trajectory", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- Service untuk save map ---
        save_map_srv_ = this->create_service<std_srvs::srv::Empty>(
            "/slam/save_map",
            std::bind(&SlamRgbdCam::saveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
            "=== 3D SLAM READY ===");
        RCLCPP_INFO(this->get_logger(),
            "  4 Kamera RGBD | GICP/NDT | OctoMap | Keyframe Management");
        RCLCPP_INFO(this->get_logger(),
            "  Save map: ros2 service call /slam/save_map std_srvs/srv/Empty");
    }

    ~SlamRgbdCam() {
        saveMap();
    }

private:
    void loadParameters() {
        max_cam_d_ = this->get_parameter("max_cam_d").as_double();
        min_cam_d_ = this->get_parameter("min_cam_d").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        map_voxel_leaf_size_ = this->get_parameter("map_voxel_leaf_size").as_double();
        octomap_resolution_ = this->get_parameter("octomap_resolution").as_double();
        gicp_max_corr_dist_ = this->get_parameter("gicp_max_corr_dist").as_double();
        gicp_max_iterations_ = this->get_parameter("gicp_max_iterations").as_int();
        gicp_transform_epsilon_ = this->get_parameter("gicp_transform_epsilon").as_double();
        gicp_fitness_threshold_ = this->get_parameter("gicp_fitness_threshold").as_double();
        keyframe_dist_threshold_ = this->get_parameter("keyframe_dist_threshold").as_double();
        keyframe_angle_threshold_ = this->get_parameter("keyframe_angle_threshold").as_double();
        local_map_size_ = this->get_parameter("local_map_size").as_int();
        crop_min_z_ = this->get_parameter("crop_min_z").as_double();
        crop_max_z_ = this->get_parameter("crop_max_z").as_double();
        self_filter_size_ = this->get_parameter("self_filter_size").as_double();
        sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
        sor_stddev_ = this->get_parameter("sor_stddev").as_double();
        use_ndt_ = this->get_parameter("use_ndt").as_bool();
        ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
        ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
        map_save_path_ = this->get_parameter("map_save_path").as_string();
        publish_octomap_cloud_ = this->get_parameter("publish_octomap_cloud").as_bool();
        sensor_height_ = this->get_parameter("sensor_height").as_double();

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

    // ======== Point Cloud Merging (4 Kamera → 1 Cloud) ========

    Eigen::Affine3f makeCameraTransform(float x, float y, float yaw) {
        // Transform kamera: posisi + yaw di base_link
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.translation() << x, y, sensor_height_;
        t.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

        // Rotasi dari optical frame (Gazebo depth camera convention)
        // Optical: Z=forward(depth), X=right, Y=down
        // Base:    X=forward, Y=left, Z=up
        //   x_base =  z_optical
        //   y_base = -x_optical
        //   z_base = -y_optical
        Eigen::Affine3f optical_to_base = Eigen::Affine3f::Identity();
        Eigen::Matrix3f R;
        R <<  0,  0, 1,
             -1,  0, 0,
              0, -1, 0;
        optical_to_base.linear() = R;

        return t * optical_to_base;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr mergeCameras(
        const sensor_msgs::msg::PointCloud2::SharedPtr& msg_c1)
    {
        auto merged = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // Cam 1 (Depan)
        auto c1_pc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg_c1, *c1_pc);
        pcl::transformPointCloud(*c1_pc, *c1_pc, makeCameraTransform(cam1_x_, cam1_y_, cam1_yaw_));
        *merged += *c1_pc;

        // Cam 2 (Belakang)
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

        return merged;
    }

    // ======== Filtering Pipeline ========

    PointCloudT::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud) {
        // 1. Voxel Grid downsampling
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(raw_cloud);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        vg.filter(*raw_cloud);

        // 2. Crop Box - batas area yang valid
        pcl::CropBox<pcl::PointXYZ> room_filter;
        room_filter.setInputCloud(raw_cloud);
        room_filter.setMin(Eigen::Vector4f(-max_cam_d_, -max_cam_d_, crop_min_z_, 1.0));
        room_filter.setMax(Eigen::Vector4f(max_cam_d_, max_cam_d_, crop_max_z_, 1.0));
        room_filter.filter(*raw_cloud);

        // 3. Self-filter - hapus titik di sekitar robot sendiri
        pcl::CropBox<pcl::PointXYZ> self_filter;
        self_filter.setInputCloud(raw_cloud);
        self_filter.setMin(Eigen::Vector4f(-self_filter_size_, -self_filter_size_, 0.1, 1.0));
        self_filter.setMax(Eigen::Vector4f(self_filter_size_, self_filter_size_, crop_max_z_, 1.0));
        self_filter.setNegative(true);
        self_filter.filter(*raw_cloud);

        // 4. Statistical Outlier Removal
        if (raw_cloud->size() > 50) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(raw_cloud);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            sor.filter(*raw_cloud);
        }

        // 5. Konversi ke PointXYZI
        auto filtered = pcl::make_shared<PointCloudT>();
        pcl::copyPointCloud(*raw_cloud, *filtered);
        for (auto& pt : filtered->points) {
            pt.intensity = 255;
        }

        return filtered;
    }

    // ======== Scan Matching (GICP / NDT) ========

    bool performScanMatching(PointCloudT::Ptr source, PointCloudT::Ptr target,
                             const Eigen::Matrix4f& initial_guess,
                             Eigen::Matrix4f& result_transform,
                             double& fitness_score)
    {
        if (source->empty() || target->empty()) return false;

        PointCloudT::Ptr aligned(new PointCloudT());

        if (use_ndt_) {
            pcl::NormalDistributionsTransform<PointT, PointT> ndt;
            ndt.setInputSource(source);
            ndt.setInputTarget(target);
            ndt.setStepSize(ndt_step_size_);
            ndt.setResolution(ndt_resolution_);
            ndt.setMaximumIterations(gicp_max_iterations_);
            ndt.setTransformationEpsilon(gicp_transform_epsilon_);
            ndt.align(*aligned, initial_guess);

            if (ndt.hasConverged()) {
                result_transform = ndt.getFinalTransformation();
                fitness_score = ndt.getFitnessScore();
                return true;
            }
        } else {
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
            gicp.setInputSource(source);
            gicp.setInputTarget(target);
            gicp.setMaxCorrespondenceDistance(gicp_max_corr_dist_);
            gicp.setMaximumIterations(gicp_max_iterations_);
            gicp.setTransformationEpsilon(gicp_transform_epsilon_);
            gicp.setEuclideanFitnessEpsilon(1e-6);
            gicp.align(*aligned, initial_guess);

            if (gicp.hasConverged()) {
                result_transform = gicp.getFinalTransformation();
                fitness_score = gicp.getFitnessScore();
                return true;
            }
        }

        return false;
    }

    // ======== Keyframe Management ========

    bool isKeyframe(const Eigen::Matrix4f& current_pose) {
        if (keyframes_.empty()) return true;

        Eigen::Matrix4f last_kf_pose = keyframe_poses_.back();
        Eigen::Vector3f delta_trans = current_pose.block<3,1>(0,3) - last_kf_pose.block<3,1>(0,3);
        float dist = delta_trans.norm();

        // Hitung perubahan sudut
        Eigen::Matrix3f delta_rot = last_kf_pose.block<3,3>(0,0).transpose() * current_pose.block<3,3>(0,0);
        float angle = std::acos(std::min(1.0f, std::max(-1.0f,
            (delta_rot.trace() - 1.0f) / 2.0f)));

        return (dist > keyframe_dist_threshold_ || angle > keyframe_angle_threshold_);
    }

    void addKeyframe(PointCloudT::Ptr cloud, const Eigen::Matrix4f& pose) {
        keyframes_.push_back(cloud);
        keyframe_poses_.push_back(pose);

        // Update local map (sliding window)
        updateLocalMap();

        RCLCPP_DEBUG(this->get_logger(), "Keyframe #%zu ditambahkan (total points local_map: %zu)",
                     keyframes_.size(), local_map_->size());
    }

    void updateLocalMap() {
        local_map_->clear();

        int start_idx = std::max(0, (int)keyframes_.size() - local_map_size_);
        for (int i = start_idx; i < (int)keyframes_.size(); ++i) {
            PointCloudT::Ptr transformed(new PointCloudT());
            pcl::transformPointCloud(*keyframes_[i], *transformed, keyframe_poses_[i]);
            *local_map_ += *transformed;
        }

        // Downsample local map
        if (local_map_->size() > 1000) {
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(local_map_);
            vg.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
            vg.filter(*local_map_);
        }
    }

    // ======== OctoMap Raycasting ========

    void performRaycasting(PointCloudT::Ptr cloud_in_map, const Eigen::Matrix4f& robot_pose_map) {
        Eigen::Vector4f sensor_origin(0, 0, sensor_height_, 1.0);
        Eigen::Vector4f sensor_in_map = robot_pose_map * sensor_origin;
        octomap::point3d origin(sensor_in_map.x(), sensor_in_map.y(), sensor_in_map.z());

        octomap::Pointcloud octo_cloud;
        for (const auto& pt : cloud_in_map->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                octo_cloud.push_back(pt.x, pt.y, pt.z);
            }
        }

        octo_map_->insertPointCloud(octo_cloud, origin, max_cam_d_, false, true);
    }

    PointCloudT::Ptr getOctoMapCloud() {
        auto cloud = pcl::make_shared<PointCloudT>();
        for (auto it = octo_map_->begin_leafs(), end = octo_map_->end_leafs(); it != end; ++it) {
            if (octo_map_->isNodeOccupied(*it)) {
                PointT pt;
                pt.x = it.getX();
                pt.y = it.getY();
                pt.z = it.getZ();
                pt.intensity = it->getOccupancy() * 255.0;
                cloud->push_back(pt);
            }
        }
        return cloud;
    }

    // ======== Map Saving ========

    void saveMap() {
        if (global_map_->empty() && keyframes_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map kosong, tidak ada yang disimpan.");
            return;
        }

        // Bangun global map dari semua keyframes
        PointCloudT::Ptr full_map(new PointCloudT());
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            PointCloudT::Ptr transformed(new PointCloudT());
            pcl::transformPointCloud(*keyframes_[i], *transformed, keyframe_poses_[i]);
            *full_map += *transformed;
        }

        // Downsample
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(full_map);
        vg.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
        vg.filter(*full_map);

        // Save PCD
        std::string pcd_path = map_save_path_ + ".pcd";
        pcl::io::savePCDFileBinaryCompressed(pcd_path, *full_map);
        RCLCPP_INFO(this->get_logger(), "Map PCD disimpan: %s (%zu points)", pcd_path.c_str(), full_map->size());

        // Save OctoMap
        std::string octo_path = map_save_path_ + ".bt";
        octo_map_->writeBinary(octo_path);
        RCLCPP_INFO(this->get_logger(), "OctoMap disimpan: %s", octo_path.c_str());

        // Save trajectory
        std::string traj_path = map_save_path_ + "_trajectory.txt";
        std::ofstream traj_file(traj_path);
        for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
            Eigen::Matrix4f& p = keyframe_poses_[i];
            traj_file << p(0,3) << " " << p(1,3) << " " << p(2,3) << " "
                      << p(0,0) << " " << p(0,1) << " " << p(0,2) << " "
                      << p(1,0) << " " << p(1,1) << " " << p(1,2) << " "
                      << p(2,0) << " " << p(2,1) << " " << p(2,2) << "\n";
        }
        traj_file.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory disimpan: %s (%zu poses)", traj_path.c_str(), keyframe_poses_.size());
    }

    void saveMapCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Save map requested via service...");
        saveMap();
    }

    // ======== Camera Callbacks ========

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

    // ======== Main SLAM Callback (Kamera 1 sebagai trigger) ========

    void cam1MainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->width * msg->height == 0 || !odom_received_) return;

        auto start_time = std::chrono::steady_clock::now();

        // 1. Merge semua kamera
        auto merged = mergeCameras(msg);
        if (merged->empty()) return;

        // 2. Filter
        auto current_cloud = filterCloud(merged);
        if (current_cloud->size() < 50) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Cloud terlalu sedikit setelah filtering: %zu points", current_cloud->size());
            return;
        }

        // 3. Transformasi ke frame odom menggunakan odometri
        Eigen::Affine3f odom_transform = Eigen::Affine3f::Identity();
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_transform.translation() << robot_x_, robot_y_, robot_z_;
            odom_transform.rotate(Eigen::Quaternionf(robot_qw_, robot_qx_, robot_qy_, robot_qz_));
        }

        PointCloudT::Ptr cloud_in_odom(new PointCloudT());
        pcl::transformPointCloud(*current_cloud, *cloud_in_odom, odom_transform);

        // 4. SLAM Processing
        if (is_first_frame_) {
            // Frame pertama: langsung jadikan keyframe
            map_to_odom_ = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f pose_in_map = map_to_odom_ * odom_transform.matrix();

            addKeyframe(current_cloud, pose_in_map);

            PointCloudT::Ptr cloud_in_map(new PointCloudT());
            pcl::transformPointCloud(*current_cloud, *cloud_in_map, pose_in_map);
            performRaycasting(cloud_in_map, pose_in_map);

            is_first_frame_ = false;
            frame_count_ = 0;

            RCLCPP_INFO(this->get_logger(), "Frame pertama diproses. SLAM dimulai!");
        } else {
            // Scan matching terhadap local map
            Eigen::Matrix4f initial_guess = map_to_odom_ * odom_transform.matrix();

            double fitness_score = 0.0;
            Eigen::Matrix4f result_transform;

            bool matched = performScanMatching(
                cloud_in_odom, local_map_, map_to_odom_, result_transform, fitness_score);

            if (matched && fitness_score < gicp_fitness_threshold_) {
                map_to_odom_ = result_transform;

                Eigen::Matrix4f pose_in_map = map_to_odom_ * odom_transform.matrix();

                // Cek apakah perlu keyframe baru
                if (isKeyframe(pose_in_map)) {
                    addKeyframe(current_cloud, pose_in_map);

                    PointCloudT::Ptr cloud_in_map(new PointCloudT());
                    pcl::transformPointCloud(*current_cloud, *cloud_in_map, pose_in_map);
                    performRaycasting(cloud_in_map, pose_in_map);
                }

                // Update pose ke path
                addPoseToPath(pose_in_map, msg->header.stamp);

                RCLCPP_DEBUG(this->get_logger(), "GICP converged. Fitness: %.4f | Keyframes: %zu",
                             fitness_score, keyframes_.size());
            } else {
                // Fallback: gunakan odometri untuk estimasi pose
                Eigen::Matrix4f pose_in_map = map_to_odom_ * odom_transform.matrix();

                // Tetap tambahkan keyframe agar map terus tumbuh
                if (isKeyframe(pose_in_map)) {
                    addKeyframe(current_cloud, pose_in_map);

                    PointCloudT::Ptr cloud_in_map(new PointCloudT());
                    pcl::transformPointCloud(*current_cloud, *cloud_in_map, pose_in_map);
                    performRaycasting(cloud_in_map, pose_in_map);
                }

                addPoseToPath(pose_in_map, msg->header.stamp);

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Scan matching fitness tinggi (%.4f > %.4f). Menggunakan odometri + tetap mapping.",
                    fitness_score, gicp_fitness_threshold_);
            }
        }

        // 5. Publish map cloud
        frame_count_++;
        if (frame_count_ % 5 == 0) {  // Publish setiap 5 frame untuk efisiensi
            PointCloudT::Ptr pub_cloud;
            if (publish_octomap_cloud_) {
                pub_cloud = getOctoMapCloud();
            } else {
                pub_cloud = local_map_;
            }

            if (!pub_cloud->empty()) {
                sensor_msgs::msg::PointCloud2 output;
                pcl::toROSMsg(*pub_cloud, output);
                output.header.stamp = msg->header.stamp;
                output.header.frame_id = "map";
                pub_map_cloud_->publish(output);
            }
        }

        // 6. Publish current scan (untuk visualisasi)
        {
            PointCloudT::Ptr scan_in_map(new PointCloudT());
            Eigen::Matrix4f pose_in_map = map_to_odom_ * odom_transform.matrix();
            pcl::transformPointCloud(*current_cloud, *scan_in_map, pose_in_map);

            sensor_msgs::msg::PointCloud2 scan_output;
            pcl::toROSMsg(*scan_in_map, scan_output);
            scan_output.header.stamp = msg->header.stamp;
            scan_output.header.frame_id = "map";
            pub_current_scan_->publish(scan_output);
        }

        // 7. Publish TF map → odom
        publishMapToOdomTF(msg->header.stamp);

        // 8. Publish path
        pub_path_->publish(path_msg_);

        auto end_time = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_DEBUG(this->get_logger(), "SLAM loop: %.1f ms", elapsed_ms);
    }

    // ======== TF & Pose Publishing ========

    void publishMapToOdomTF(const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped t_map;
        t_map.header.stamp = stamp;
        t_map.header.frame_id = "map";
        t_map.child_frame_id = "odom";

        Eigen::Affine3f affine_map(map_to_odom_);
        Eigen::Quaternionf q_map(affine_map.rotation());

        t_map.transform.translation.x = affine_map.translation().x();
        t_map.transform.translation.y = affine_map.translation().y();
        t_map.transform.translation.z = affine_map.translation().z();
        t_map.transform.rotation.x = q_map.x();
        t_map.transform.rotation.y = q_map.y();
        t_map.transform.rotation.z = q_map.z();
        t_map.transform.rotation.w = q_map.w();

        tf_broadcaster_->sendTransform(t_map);
    }

    void addPoseToPath(const Eigen::Matrix4f& pose, const rclcpp::Time& stamp) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = pose(0, 3);
        ps.pose.position.y = pose(1, 3);
        ps.pose.position.z = pose(2, 3);

        Eigen::Quaternionf q(Eigen::Matrix3f(pose.block<3,3>(0,0)));
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();

        path_msg_.poses.push_back(ps);
        pub_pose_->publish(ps);
    }

    // ======== Odometry Callback ========

    void robotOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

    // ======== Member Variables ========

    // Parameters
    double max_cam_d_, min_cam_d_;
    double voxel_leaf_size_, map_voxel_leaf_size_;
    double octomap_resolution_;
    double gicp_max_corr_dist_;
    int gicp_max_iterations_;
    double gicp_transform_epsilon_, gicp_fitness_threshold_;
    double keyframe_dist_threshold_, keyframe_angle_threshold_;
    int local_map_size_;
    double crop_min_z_, crop_max_z_;
    double self_filter_size_;
    int sor_mean_k_;
    double sor_stddev_;
    bool use_ndt_;
    double ndt_step_size_, ndt_resolution_;
    std::string map_save_path_;
    bool publish_octomap_cloud_;
    double sensor_height_;

    // Camera positions
    double cam1_x_, cam1_y_, cam1_yaw_;
    double cam2_x_, cam2_y_, cam2_yaw_;
    double cam3_x_, cam3_y_, cam3_yaw_;
    double cam4_x_, cam4_y_, cam4_yaw_;

    // Odometry state
    float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
    float robot_qx_ = 0, robot_qy_ = 0, robot_qz_ = 0, robot_qw_ = 1;
    bool odom_received_ = false;
    std::mutex odom_mutex_;

    // Camera buffers
    pcl::PointCloud<pcl::PointXYZ>::Ptr c2_, c3_, c4_;
    std::mutex cam_mutex_;

    // SLAM state
    bool is_first_frame_ = true;
    Eigen::Matrix4f map_to_odom_ = Eigen::Matrix4f::Identity();
    PointCloudT::Ptr global_map_;
    PointCloudT::Ptr local_map_;
    std::shared_ptr<octomap::OcTree> octo_map_;

    // Keyframe data
    std::vector<PointCloudT::Ptr> keyframes_;
    std::vector<Eigen::Matrix4f> keyframe_poses_;

    // Path trajectory
    nav_msgs::msg::Path path_msg_;
    int frame_count_ = 0;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_c1_, sub_c2_, sub_c3_, sub_c4_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_scan_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamRgbdCam>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
