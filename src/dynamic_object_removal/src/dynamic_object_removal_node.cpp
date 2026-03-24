#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <deque>
#include <unordered_map>
#include <cmath>
#include <string>

class DynamicObjectRemoval : public rclcpp::Node
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    DynamicObjectRemoval()
    : Node("dynamic_object_removal")
    {
        // Declare parameters
        declare_parameter("input_topic", std::string("/full_pointcloud"));
        declare_parameter("output_topic", std::string("/filtered_pointcloud"));
        declare_parameter("octree_resolution", 0.1);
        declare_parameter("noise_filter_min_neighbors", 5);
        declare_parameter("voxel_size", 0.05);
        declare_parameter("scan_min_range", 0.5);
        declare_parameter("scan_max_range", 4.0);
        declare_parameter("dynamic_removal_enabled", true);
        declare_parameter("buffer_size", 3);

        // Read parameters
        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        octree_resolution_ = get_parameter("octree_resolution").as_double();
        noise_filter_min_neighbors_ = get_parameter("noise_filter_min_neighbors").as_int();
        voxel_size_ = get_parameter("voxel_size").as_double();
        scan_min_range_ = get_parameter("scan_min_range").as_double();
        scan_max_range_ = get_parameter("scan_max_range").as_double();
        dynamic_removal_enabled_ = get_parameter("dynamic_removal_enabled").as_bool();
        buffer_size_ = get_parameter("buffer_size").as_int();

        // Publisher and subscriber
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DynamicObjectRemoval::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Dynamic object removal node started. Input: %s, Output: %s, Enabled: %s",
            input_topic_.c_str(), output_topic_.c_str(),
            dynamic_removal_enabled_ ? "true" : "false");
    }

private:
    // Voxel key for the occupancy buffer
    struct VoxelKey
    {
        int x, y, z;
        bool operator==(const VoxelKey & other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash
    {
        std::size_t operator()(const VoxelKey & k) const
        {
            std::size_t h1 = std::hash<int>()(k.x);
            std::size_t h2 = std::hash<int>()(k.y);
            std::size_t h3 = std::hash<int>()(k.z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL cloud
        PointCloudT::Ptr cloud_in(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud_in);

        if (cloud_in->empty()) {
            return;
        }

        // Step 1: Range filter
        PointCloudT::Ptr cloud_ranged(new PointCloudT);
        applyRangeFilter(cloud_in, cloud_ranged);

        // Step 2: Voxel grid downsampling
        PointCloudT::Ptr cloud_downsampled(new PointCloudT);
        applyVoxelFilter(cloud_ranged, cloud_downsampled);

        if (!dynamic_removal_enabled_) {
            // Pass through with only range + voxel filtering
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_downsampled, output_msg);
            output_msg.header = msg->header;
            pub_->publish(output_msg);
            return;
        }

        // Step 3: Octree change detection and occupancy buffer
        PointCloudT::Ptr cloud_static(new PointCloudT);
        detectAndRemoveDynamic(cloud_downsampled, cloud_static);

        // Publish filtered cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_static, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);

        RCLCPP_DEBUG(get_logger(),
            "In: %zu, Ranged: %zu, Downsampled: %zu, Static: %zu",
            cloud_in->size(), cloud_ranged->size(),
            cloud_downsampled->size(), cloud_static->size());
    }

    void applyRangeFilter(
        const PointCloudT::Ptr & input,
        PointCloudT::Ptr & output)
    {
        output->points.reserve(input->size());
        for (const auto & pt : input->points) {
            double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range >= scan_min_range_ && range <= scan_max_range_) {
                output->points.push_back(pt);
            }
        }
        output->width = output->points.size();
        output->height = 1;
        output->is_dense = true;
    }

    void applyVoxelFilter(
        const PointCloudT::Ptr & input,
        PointCloudT::Ptr & output)
    {
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_));
        voxel.filter(*output);
    }

    VoxelKey pointToVoxelKey(const PointT & pt) const
    {
        return VoxelKey{
            static_cast<int>(std::floor(pt.x / octree_resolution_)),
            static_cast<int>(std::floor(pt.y / octree_resolution_)),
            static_cast<int>(std::floor(pt.z / octree_resolution_))
        };
    }

    void detectAndRemoveDynamic(
        const PointCloudT::Ptr & input,
        PointCloudT::Ptr & output)
    {
        // Build a set of voxel keys occupied in the current frame
        std::unordered_map<VoxelKey, bool, VoxelKeyHash> current_occupied;
        for (const auto & pt : input->points) {
            current_occupied[pointToVoxelKey(pt)] = true;
        }

        // Push current occupancy into the ring buffer
        occupancy_buffer_.push_back(current_occupied);
        if (static_cast<int>(occupancy_buffer_.size()) > buffer_size_) {
            occupancy_buffer_.pop_front();
        }

        // If we haven't accumulated enough frames yet, pass everything through
        if (static_cast<int>(occupancy_buffer_.size()) < buffer_size_) {
            *output = *input;
            return;
        }

        // A voxel is static if it was occupied in ALL frames in the buffer.
        // Count occupancy across the buffer for each voxel in the current frame.
        output->points.reserve(input->size());
        for (const auto & pt : input->points) {
            VoxelKey key = pointToVoxelKey(pt);

            int count = 0;
            for (const auto & frame_map : occupancy_buffer_) {
                if (frame_map.find(key) != frame_map.end()) {
                    count++;
                }
            }

            // Keep point only if voxel was occupied in all buffered frames (static)
            if (count >= buffer_size_) {
                output->points.push_back(pt);
            }
        }

        output->width = output->points.size();
        output->height = 1;
        output->is_dense = true;
    }

    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    double octree_resolution_;
    int noise_filter_min_neighbors_;
    double voxel_size_;
    double scan_min_range_;
    double scan_max_range_;
    bool dynamic_removal_enabled_;
    int buffer_size_;

    // Occupancy ring buffer: each entry is a map of occupied voxel keys for one frame
    std::deque<std::unordered_map<VoxelKey, bool, VoxelKeyHash>> occupancy_buffer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicObjectRemoval>());
    rclcpp::shutdown();
    return 0;
}
