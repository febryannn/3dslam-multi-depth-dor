#include <memory>
#include <chrono>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include <future>
#include <fstream>

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
#include <tf2/convert.h>
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

using namespace std::chrono_literals;

/**
 * 3D SLAM Node — Arsitektur scan-to-submap mengikuti lidarslam_ros2
 *
 * Mengikuti pattern ScanMatcherComponent dari lidarslam_ros2:
 *   1. Receive point cloud (dari pointcloud_concatenate atau langsung)
 *   2. Transform ke robot_frame jika diperlukan
 *   3. Filter (min/max range, voxel grid)
 *   4. GICP/NDT scan-to-submap matching
 *   5. Publish pose, path, TF (map->odom atau map->base_link)
 *   6. Update submap saat robot bergerak cukup jauh (background thread)
 *   7. Publish full map secara periodik
 */
class SlamRgbdCam : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    SlamRgbdCam(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("slam_rgbd_cam", options),
      clock_(RCL_ROS_TIME),
      tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
      listener_(tfbuffer_),
      broadcaster_(this)
    {
        RCLCPP_INFO(get_logger(), "initialization start");

        // --- Declare & get parameters ---
        double ndt_resolution;
        double gicp_corr_dist_threshold;

        declare_parameter("global_frame_id", "map");
        get_parameter("global_frame_id", global_frame_id_);
        declare_parameter("robot_frame_id", "base_link");
        get_parameter("robot_frame_id", robot_frame_id_);
        declare_parameter("odom_frame_id", "odom");
        get_parameter("odom_frame_id", odom_frame_id_);
        declare_parameter("registration_method", "GICP");
        get_parameter("registration_method", registration_method_);
        declare_parameter("ndt_resolution", 5.0);
        get_parameter("ndt_resolution", ndt_resolution);
        declare_parameter("gicp_corr_dist_threshold", 5.0);
        get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);
        declare_parameter("trans_for_mapupdate", 1.5);
        get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
        declare_parameter("vg_size_for_input", 0.2);
        get_parameter("vg_size_for_input", vg_size_for_input_);
        declare_parameter("vg_size_for_map", 0.1);
        get_parameter("vg_size_for_map", vg_size_for_map_);
        declare_parameter("use_min_max_filter", false);
        get_parameter("use_min_max_filter", use_min_max_filter_);
        declare_parameter("scan_min_range", 0.1);
        get_parameter("scan_min_range", scan_min_range_);
        declare_parameter("scan_max_range", 100.0);
        get_parameter("scan_max_range", scan_max_range_);
        declare_parameter("map_publish_period", 15.0);
        get_parameter("map_publish_period", map_publish_period_);
        declare_parameter("num_targeted_cloud", 10);
        get_parameter("num_targeted_cloud", num_targeted_cloud_);

        if (num_targeted_cloud_ < 1) {
            RCLCPP_WARN(get_logger(), "num_targeted_cloud should be positive, setting to 1");
            num_targeted_cloud_ = 1;
        }

        declare_parameter("initial_pose_x", 0.0);
        get_parameter("initial_pose_x", initial_pose_x_);
        declare_parameter("initial_pose_y", 0.0);
        get_parameter("initial_pose_y", initial_pose_y_);
        declare_parameter("initial_pose_z", 0.0);
        get_parameter("initial_pose_z", initial_pose_z_);
        declare_parameter("initial_pose_qx", 0.0);
        get_parameter("initial_pose_qx", initial_pose_qx_);
        declare_parameter("initial_pose_qy", 0.0);
        get_parameter("initial_pose_qy", initial_pose_qy_);
        declare_parameter("initial_pose_qz", 0.0);
        get_parameter("initial_pose_qz", initial_pose_qz_);
        declare_parameter("initial_pose_qw", 1.0);
        get_parameter("initial_pose_qw", initial_pose_qw_);
        declare_parameter("set_initial_pose", false);
        get_parameter("set_initial_pose", set_initial_pose_);
        declare_parameter("publish_tf", true);
        get_parameter("publish_tf", publish_tf_);
        declare_parameter("use_odom", false);
        get_parameter("use_odom", use_odom_);
        declare_parameter("debug_flag", false);
        get_parameter("debug_flag", debug_flag_);

        declare_parameter("input_cloud_topic", "/full_pointcloud");
        get_parameter("input_cloud_topic", input_cloud_topic_);
        declare_parameter("map_save_path", "/tmp/slam_map");
        get_parameter("map_save_path", map_save_path_);

        // Print parameters
        std::cout << "=== SLAM Parameters ===" << std::endl;
        std::cout << "registration_method: " << registration_method_ << std::endl;
        std::cout << "ndt_resolution[m]: " << ndt_resolution << std::endl;
        std::cout << "gicp_corr_dist_threshold[m]: " << gicp_corr_dist_threshold << std::endl;
        std::cout << "trans_for_mapupdate[m]: " << trans_for_mapupdate_ << std::endl;
        std::cout << "vg_size_for_input[m]: " << vg_size_for_input_ << std::endl;
        std::cout << "vg_size_for_map[m]: " << vg_size_for_map_ << std::endl;
        std::cout << "use_min_max_filter: " << std::boolalpha << use_min_max_filter_ << std::endl;
        std::cout << "scan_min_range[m]: " << scan_min_range_ << std::endl;
        std::cout << "scan_max_range[m]: " << scan_max_range_ << std::endl;
        std::cout << "set_initial_pose: " << std::boolalpha << set_initial_pose_ << std::endl;
        std::cout << "publish_tf: " << std::boolalpha << publish_tf_ << std::endl;
        std::cout << "use_odom: " << std::boolalpha << use_odom_ << std::endl;
        std::cout << "debug_flag: " << std::boolalpha << debug_flag_ << std::endl;
        std::cout << "map_publish_period[sec]: " << map_publish_period_ << std::endl;
        std::cout << "num_targeted_cloud: " << num_targeted_cloud_ << std::endl;
        std::cout << "input_cloud_topic: " << input_cloud_topic_ << std::endl;
        std::cout << "=======================" << std::endl;

        // --- Registration algorithm setup ---
        if (registration_method_ == "NDT") {
            auto ndt = std::make_shared<pcl::NormalDistributionsTransform<PointT, PointT>>();
            ndt->setResolution(ndt_resolution);
            ndt->setTransformationEpsilon(0.01);
            ndt->setMaximumIterations(50);
            registration_ = ndt;
        } else if (registration_method_ == "GICP") {
            auto gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>();
            gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
            gicp->setTransformationEpsilon(1e-8);
            gicp->setMaximumIterations(64);
            gicp->setEuclideanFitnessEpsilon(1e-6);
            registration_ = gicp;
        } else {
            RCLCPP_ERROR(get_logger(), "invalid registration method: %s", registration_method_.c_str());
            return;
        }

        // --- Init state ---
        path_.header.frame_id = global_frame_id_;
        previous_odom_mat_ = Eigen::Matrix4f::Identity();

        // --- Initialize publishers & subscribers ---
        initializePubSub();

        // --- Set initial pose if configured ---
        if (set_initial_pose_) {
            RCLCPP_INFO(get_logger(), "set initial pose");
            auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
            msg->header.stamp = now();
            msg->header.frame_id = global_frame_id_;
            msg->pose.position.x = initial_pose_x_;
            msg->pose.position.y = initial_pose_y_;
            msg->pose.position.z = initial_pose_z_;
            msg->pose.orientation.x = initial_pose_qx_;
            msg->pose.orientation.y = initial_pose_qy_;
            msg->pose.orientation.z = initial_pose_qz_;
            msg->pose.orientation.w = initial_pose_qw_;
            current_pose_stamped_ = *msg;
            pose_pub_->publish(current_pose_stamped_);
            initial_pose_received_ = true;
            path_.poses.push_back(*msg);
        }

        RCLCPP_INFO(get_logger(), "initialization end");
    }

    ~SlamRgbdCam() {
        saveMap();
        if (mapping_thread_.joinable()) {
            mapping_thread_.detach();
        }
    }

private:
    // =============== SubMap structure ===============
    struct SubMap {
        std_msgs::msg::Header header;
        double distance;
        geometry_msgs::msg::Pose pose;
        sensor_msgs::msg::PointCloud2 cloud;  // local frame cloud (as ROS msg)
    };

    // =============== Initialize Publishers/Subscribers ===============
    void initializePubSub() {
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");

        // --- Initial pose subscriber ---
        auto initial_pose_callback =
            [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
                if (msg->header.frame_id != global_frame_id_) {
                    RCLCPP_WARN(get_logger(), "initial_pose is not in the global frame");
                    return;
                }
                RCLCPP_INFO(get_logger(), "initial_pose is received");
                current_pose_stamped_ = *msg;
                previous_position_.x() = current_pose_stamped_.pose.position.x;
                previous_position_.y() = current_pose_stamped_.pose.position.y;
                previous_position_.z() = current_pose_stamped_.pose.position.z;
                initial_pose_received_ = true;
                pose_pub_->publish(current_pose_stamped_);
            };

        // --- Cloud callback (main SLAM pipeline) ---
        auto cloud_callback =
            [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
                if (!initial_pose_received_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "initial_pose is not received, waiting...");
                    return;
                }

                PointCloudT::Ptr tmp_ptr(new PointCloudT());
                pcl::fromROSMsg(*msg, *tmp_ptr);
                if (tmp_ptr->empty()) return;

                // Transform to robot_frame if needed
                if (msg->header.frame_id != robot_frame_id_) {
                    try {
                        tf2::TimePoint time_point = tf2::TimePoint(
                            std::chrono::seconds(msg->header.stamp.sec) +
                            std::chrono::nanoseconds(msg->header.stamp.nanosec));
                        const geometry_msgs::msg::TransformStamped transform =
                            tfbuffer_.lookupTransform(
                                robot_frame_id_, msg->header.frame_id, time_point);
                        Eigen::Affine3d affine = tf2::transformToEigen(transform);
                        PointCloudT::Ptr transformed(new PointCloudT());
                        pcl::transformPointCloud(*tmp_ptr, *transformed,
                                                affine.matrix().cast<float>());
                        tmp_ptr = transformed;
                    } catch (tf2::TransformException & e) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "TF transform failed: %s", e.what());
                        return;
                    }
                }

                // Min/max range filter
                if (use_min_max_filter_) {
                    PointCloudT::Ptr filtered(new PointCloudT());
                    for (const auto & p : tmp_ptr->points) {
                        double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                        if (scan_min_range_ < r && r < scan_max_range_) {
                            filtered->points.push_back(p);
                        }
                    }
                    tmp_ptr = filtered;
                }

                if (tmp_ptr->size() < 30) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Cloud too small: %zu points", tmp_ptr->size());
                    return;
                }

                if (!initial_cloud_received_) {
                    RCLCPP_INFO(get_logger(), "initial cloud is received");
                    initial_cloud_received_ = true;
                    initializeMap(tmp_ptr, msg->header);
                    last_map_time_ = clock_.now();
                }

                if (initial_cloud_received_) {
                    receiveCloud(tmp_ptr, msg->header.stamp);
                }
            };

        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "initial_pose", rclcpp::QoS(10), initial_pose_callback);

        input_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, rclcpp::SensorDataQoS(), cloud_callback);

        // Publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "current_pose", rclcpp::QoS(10));
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "map", rclcpp::QoS(10));
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "path", rclcpp::QoS(10));

        // Save map service
        save_map_srv_ = create_service<std_srvs::srv::Empty>(
            "save_map",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
                RCLCPP_INFO(get_logger(), "Save map requested...");
                saveMap();
            });
    }

    // =============== Initialize Map (first frame) ===============
    void initializeMap(const PointCloudT::Ptr & cloud_ptr,
                       const std_msgs::msg::Header & header) {
        RCLCPP_INFO(get_logger(), "create a first map");

        // Downsample for map storage
        PointCloudT::Ptr map_cloud(new PointCloudT());
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
        voxel_grid.setInputCloud(cloud_ptr);
        voxel_grid.filter(*map_cloud);

        // Transform to global frame using current pose
        Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);
        PointCloudT::Ptr transformed_cloud(new PointCloudT());
        pcl::transformPointCloud(*map_cloud, *transformed_cloud, sim_trans);

        // Set as registration target
        registration_->setInputTarget(transformed_cloud);

        // Store as first submap
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*map_cloud, cloud_msg);

        SubMap submap;
        submap.header = header;
        submap.distance = 0;
        submap.pose = current_pose_stamped_.pose;
        submap.cloud = cloud_msg;

        submaps_.push_back(submap);

        // Publish initial map
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*transformed_cloud, map_msg);
        map_msg.header.frame_id = global_frame_id_;
        map_msg.header.stamp = header.stamp;
        map_pub_->publish(map_msg);

        RCLCPP_INFO(get_logger(), "First map created (%zu pts). SLAM active.",
                    map_cloud->size());
    }

    // =============== Receive Cloud (main SLAM loop) ===============
    void receiveCloud(const PointCloudT::Ptr & cloud_ptr,
                      const rclcpp::Time stamp) {
        // Check if background map update is done
        if (mapping_flag_ && mapping_future_.valid()) {
            auto status = mapping_future_.wait_for(0s);
            if (status == std::future_status::ready) {
                if (is_map_updated_) {
                    PointCloudT::Ptr targeted_cloud_ptr(
                        new PointCloudT(targeted_cloud_));
                    if (registration_method_ == "NDT") {
                        registration_->setInputTarget(targeted_cloud_ptr);
                    } else {
                        // For GICP, downsample the target
                        PointCloudT::Ptr filtered(new PointCloudT());
                        pcl::VoxelGrid<PointT> vg;
                        vg.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
                        vg.setInputCloud(targeted_cloud_ptr);
                        vg.filter(*filtered);
                        registration_->setInputTarget(filtered);
                    }
                    is_map_updated_ = false;
                }
                mapping_flag_ = false;
                if (mapping_thread_.joinable()) {
                    mapping_thread_.detach();
                }
            }
        }

        // Voxel grid filter input cloud
        PointCloudT::Ptr filtered_cloud(new PointCloudT());
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
        voxel_grid.setInputCloud(cloud_ptr);
        voxel_grid.filter(*filtered_cloud);

        registration_->setInputSource(filtered_cloud);

        // Compute initial guess
        Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);

        if (use_odom_) {
            geometry_msgs::msg::TransformStamped odom_trans;
            try {
                odom_trans = tfbuffer_.lookupTransform(
                    odom_frame_id_, robot_frame_id_,
                    tf2_ros::fromMsg(stamp));
            } catch (tf2::TransformException & e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Odom TF lookup failed: %s", e.what());
            }
            Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_trans);
            Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();

            if (previous_odom_mat_ != Eigen::Matrix4f::Identity()) {
                sim_trans = sim_trans * previous_odom_mat_.inverse() * odom_mat;
            }
            previous_odom_mat_ = odom_mat;
        }

        // Align (scan matching)
        PointCloudT::Ptr output_cloud(new PointCloudT());
        rclcpp::Clock system_clock;
        rclcpp::Time time_align_start = system_clock.now();
        registration_->align(*output_cloud, sim_trans);
        rclcpp::Time time_align_end = system_clock.now();

        Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

        // Publish pose, TF, and check map update
        publishMapAndPose(cloud_ptr, final_transformation, stamp);

        // Debug output
        if (debug_flag_) {
            double roll, pitch, yaw;
            auto & o = current_pose_stamped_.pose.orientation;
            tf2::Quaternion quat_tf(o.x, o.y, o.z, o.w);
            tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

            std::cout << "---------------------------------------------------------" << std::endl;
            std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
            std::cout << "trans: " << trans_ << std::endl;
            std::cout << "align time: " << time_align_end.seconds() - time_align_start.seconds()
                      << "s" << std::endl;
            std::cout << "number of filtered cloud points: " << filtered_cloud->size() << std::endl;
            std::cout << "initial transformation:" << std::endl;
            std::cout << sim_trans << std::endl;
            std::cout << "has converged: " << registration_->hasConverged() << std::endl;
            std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
            std::cout << "final transformation:" << std::endl;
            std::cout << final_transformation << std::endl;
            std::cout << "rpy: roll=" << roll * 180 / M_PI
                      << ", pitch=" << pitch * 180 / M_PI
                      << ", yaw=" << yaw * 180 / M_PI << std::endl;
            std::cout << "num_submaps: " << submaps_.size() << std::endl;
            std::cout << "moving distance: " << latest_distance_ << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl;
        }
    }

    // =============== Publish Map, Pose and TF ===============
    void publishMapAndPose(const PointCloudT::Ptr & cloud_ptr,
                           const Eigen::Matrix4f final_transformation,
                           const rclcpp::Time stamp) {
        Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();
        Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond quat_eig(rot_mat);
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

        // Publish TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped base_to_map_msg;
            base_to_map_msg.header.stamp = stamp;
            base_to_map_msg.header.frame_id = global_frame_id_;
            base_to_map_msg.child_frame_id = robot_frame_id_;
            base_to_map_msg.transform.translation.x = position.x();
            base_to_map_msg.transform.translation.y = position.y();
            base_to_map_msg.transform.translation.z = position.z();
            base_to_map_msg.transform.rotation = quat_msg;

            if (use_odom_) {
                // Calculate map->odom transform
                geometry_msgs::msg::TransformStamped odom_to_map_msg;
                odom_to_map_msg = calculateMapToOdomTransform(base_to_map_msg, stamp);
                broadcaster_.sendTransform(odom_to_map_msg);
            } else {
                // Direct map->base_link
                broadcaster_.sendTransform(base_to_map_msg);
            }
        }

        // Update and publish current pose
        current_pose_stamped_.header.stamp = stamp;
        current_pose_stamped_.header.frame_id = global_frame_id_;
        current_pose_stamped_.pose.position.x = position.x();
        current_pose_stamped_.pose.position.y = position.y();
        current_pose_stamped_.pose.position.z = position.z();
        current_pose_stamped_.pose.orientation = quat_msg;
        pose_pub_->publish(current_pose_stamped_);

        // Update path
        path_.poses.push_back(current_pose_stamped_);
        path_pub_->publish(path_);

        // Check if we need to create a new submap
        trans_ = (position - previous_position_).norm();
        if (trans_ >= trans_for_mapupdate_ && !mapping_flag_) {
            geometry_msgs::msg::PoseStamped current_pose_copy = current_pose_stamped_;
            previous_position_ = position;

            mapping_task_ = std::packaged_task<void()>(
                std::bind(&SlamRgbdCam::updateMap, this,
                          cloud_ptr, final_transformation, current_pose_copy));
            mapping_future_ = mapping_task_.get_future();
            mapping_thread_ = std::thread(std::move(std::ref(mapping_task_)));
            mapping_flag_ = true;
        }
    }

    // =============== Calculate map->odom Transform ===============
    // map_T_odom = map_T_base * inv(odom_T_base)
    // Uses pure Eigen math to avoid tf2_geometry_msgs doTransform linkage issues
    geometry_msgs::msg::TransformStamped calculateMapToOdomTransform(
        const geometry_msgs::msg::TransformStamped & base_to_map_msg,
        const rclcpp::Time stamp)
    {
        geometry_msgs::msg::TransformStamped odom_to_map_msg;
        try {
            // Get odom->base_link transform
            auto odom_base_tf = tfbuffer_.lookupTransform(
                odom_frame_id_, robot_frame_id_, tf2::TimePointZero);
            Eigen::Affine3d odom_T_base = tf2::transformToEigen(odom_base_tf);

            // Get map->base_link from the SLAM result
            Eigen::Affine3d map_T_base = tf2::transformToEigen(base_to_map_msg);

            // map_T_odom = map_T_base * inv(odom_T_base)
            Eigen::Affine3d map_T_odom = map_T_base * odom_T_base.inverse();
            Eigen::Quaterniond q(map_T_odom.rotation());

            odom_to_map_msg.header.stamp = stamp;
            odom_to_map_msg.header.frame_id = global_frame_id_;
            odom_to_map_msg.child_frame_id = odom_frame_id_;
            odom_to_map_msg.transform.translation.x = map_T_odom.translation().x();
            odom_to_map_msg.transform.translation.y = map_T_odom.translation().y();
            odom_to_map_msg.transform.translation.z = map_T_odom.translation().z();
            odom_to_map_msg.transform.rotation.x = q.x();
            odom_to_map_msg.transform.rotation.y = q.y();
            odom_to_map_msg.transform.rotation.z = q.z();
            odom_to_map_msg.transform.rotation.w = q.w();
        } catch (tf2::TransformException & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                "Transform from base_link to odom failed: %s", e.what());
        }

        return odom_to_map_msg;
    }

    // =============== Update Map (Background Thread) ===============
    void updateMap(const PointCloudT::Ptr cloud_ptr,
                   const Eigen::Matrix4f final_transformation,
                   const geometry_msgs::msg::PoseStamped current_pose_stamped) {
        // Downsample for map storage
        PointCloudT::Ptr filtered_cloud(new PointCloudT());
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
        voxel_grid.setInputCloud(cloud_ptr);
        voxel_grid.filter(*filtered_cloud);

        // Transform to global frame
        PointCloudT::Ptr transformed_cloud(new PointCloudT());
        pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, final_transformation);

        // Build targeted cloud from new + N-1 recent submaps
        targeted_cloud_.clear();
        targeted_cloud_ += *transformed_cloud;

        int num_submaps = submaps_.size();
        for (int i = 0; i < num_targeted_cloud_ - 1; i++) {
            int idx = num_submaps - 1 - i;
            if (idx < 0) continue;

            PointCloudT::Ptr tmp_ptr(new PointCloudT());
            pcl::fromROSMsg(submaps_[idx].cloud, *tmp_ptr);

            PointCloudT::Ptr transformed_tmp(new PointCloudT());
            Eigen::Affine3d submap_affine;
            tf2::fromMsg(submaps_[idx].pose, submap_affine);
            pcl::transformPointCloud(*tmp_ptr, *transformed_tmp,
                                    submap_affine.matrix().cast<float>());
            targeted_cloud_ += *transformed_tmp;
        }

        // Store new submap
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*filtered_cloud, cloud_msg);

        SubMap submap;
        submap.header.frame_id = global_frame_id_;
        submap.header.stamp = current_pose_stamped.header.stamp;
        latest_distance_ += trans_;
        submap.distance = latest_distance_;
        submap.pose = current_pose_stamped.pose;
        submap.cloud = cloud_msg;
        submap.cloud.header.frame_id = global_frame_id_;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            submaps_.push_back(submap);
        }

        is_map_updated_ = true;

        // Periodically publish full map
        rclcpp::Time map_time = clock_.now();
        double dt = map_time.seconds() - last_map_time_.seconds();
        if (dt > map_publish_period_) {
            publishFullMap();
            last_map_time_ = map_time;
        }

        RCLCPP_INFO(get_logger(), "Submap #%zu (dist: %.2fm, target: %zu pts)",
                    submaps_.size(), submap.distance, targeted_cloud_.size());
    }

    // =============== Utility: Pose -> Matrix4f ===============
    Eigen::Matrix4f getTransformation(const geometry_msgs::msg::Pose & pose) {
        Eigen::Affine3d affine;
        tf2::fromMsg(pose, affine);
        return affine.matrix().cast<float>();
    }

    // =============== Publish Full Map ===============
    void publishFullMap() {
        PointCloudT::Ptr map_ptr(new PointCloudT());
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            for (const auto & submap : submaps_) {
                PointCloudT::Ptr submap_cloud(new PointCloudT());
                PointCloudT::Ptr transformed(new PointCloudT());
                pcl::fromROSMsg(submap.cloud, *submap_cloud);
                Eigen::Affine3d affine;
                tf2::fromMsg(submap.pose, affine);
                pcl::transformPointCloud(*submap_cloud, *transformed,
                                        affine.matrix().cast<float>());
                *map_ptr += *transformed;
            }
        }

        if (map_ptr->empty()) return;

        RCLCPP_INFO(get_logger(), "publish map, points: %zu", map_ptr->size());
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_ptr, map_msg);
        map_msg.header.frame_id = global_frame_id_;
        map_msg.header.stamp = now();
        map_pub_->publish(map_msg);
    }

    // =============== Save Map ===============
    void saveMap() {
        if (submaps_.empty()) {
            RCLCPP_WARN(get_logger(), "Map is empty, nothing to save.");
            return;
        }

        PointCloudT::Ptr full(new PointCloudT());
        for (const auto & submap : submaps_) {
            PointCloudT::Ptr cloud(new PointCloudT());
            PointCloudT::Ptr transformed(new PointCloudT());
            pcl::fromROSMsg(submap.cloud, *cloud);
            Eigen::Affine3d affine;
            tf2::fromMsg(submap.pose, affine);
            pcl::transformPointCloud(*cloud, *transformed,
                                    affine.matrix().cast<float>());
            *full += *transformed;
        }

        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(full);
        vg.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
        vg.filter(*full);

        std::string pcd_path = map_save_path_ + ".pcd";
        pcl::io::savePCDFileBinaryCompressed(pcd_path, *full);
        RCLCPP_INFO(get_logger(), "Map saved: %s (%zu pts)", pcd_path.c_str(), full->size());

        // Save trajectory
        std::string traj_path = map_save_path_ + "_trajectory.txt";
        std::ofstream f(traj_path);
        for (const auto & sm : submaps_) {
            Eigen::Affine3d affine;
            tf2::fromMsg(sm.pose, affine);
            Eigen::Matrix4d m = affine.matrix();
            f << m(0,3) << " " << m(1,3) << " " << m(2,3) << " "
              << m(0,0) << " " << m(0,1) << " " << m(0,2) << " "
              << m(1,0) << " " << m(1,1) << " " << m(1,2) << " "
              << m(2,0) << " " << m(2,1) << " " << m(2,2) << "\n";
        }
        f.close();
        RCLCPP_INFO(get_logger(), "Trajectory saved: %s (%zu poses)",
                    traj_path.c_str(), submaps_.size());
    }

    // =============== Member Variables ===============
    // Parameters
    std::string registration_method_;
    std::string global_frame_id_, robot_frame_id_, odom_frame_id_;
    std::string input_cloud_topic_, map_save_path_;
    double trans_for_mapupdate_;
    double vg_size_for_input_, vg_size_for_map_;
    bool use_min_max_filter_;
    double scan_min_range_, scan_max_range_;
    double map_publish_period_;
    int num_targeted_cloud_;
    double initial_pose_x_, initial_pose_y_, initial_pose_z_;
    double initial_pose_qx_, initial_pose_qy_, initial_pose_qz_, initial_pose_qw_;
    bool set_initial_pose_;
    bool publish_tf_;
    bool use_odom_;
    bool debug_flag_;

    // Registration
    pcl::Registration<PointT, PointT>::Ptr registration_;

    // TF2
    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    // State
    bool initial_pose_received_ = false;
    bool initial_cloud_received_ = false;
    geometry_msgs::msg::PoseStamped current_pose_stamped_;
    Eigen::Vector3d previous_position_{Eigen::Vector3d::Zero()};
    Eigen::Matrix4f previous_odom_mat_;
    double trans_ = 0.0;
    double latest_distance_ = 0.0;

    // Submaps
    std::vector<SubMap> submaps_;
    PointCloudT targeted_cloud_;
    std::mutex map_mutex_;

    // Background mapping
    bool mapping_flag_ = false;
    bool is_map_updated_ = false;
    std::packaged_task<void()> mapping_task_;
    std::future<void> mapping_future_;
    std::thread mapping_thread_;

    // Timing
    rclcpp::Time last_map_time_;

    // Path
    nav_msgs::msg::Path path_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<SlamRgbdCam>(options));
    rclcpp::shutdown();
    return 0;
}
