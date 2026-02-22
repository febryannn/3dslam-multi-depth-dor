#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>
#include <cmath>

struct PointXY {
    int x;
    int y;
};

class PointCloudToGridNode : public rclcpp::Node
{
public:
    PointCloudToGridNode() : Node("pointcloud_to_grid_node")
    {
        // Declare parameters
        this->declare_parameter("cell_size", 0.1);
        this->declare_parameter("position_x", 0.0);
        this->declare_parameter("position_y", 0.0);
        this->declare_parameter("length_x", 20.0);
        this->declare_parameter("length_y", 20.0);
        this->declare_parameter("intensity_factor", 100.0);
        this->declare_parameter("height_factor", 100.0);
        this->declare_parameter("cloud_in_topic", "pointcloud");
        this->declare_parameter("mapi_topic_name", "intensity_grid");
        this->declare_parameter("maph_topic_name", "height_grid");
        
        // Get parameters
        updateParameters();
        
        // Publishers
        pub_igrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            mapi_topic_name_, 10);
        pub_hgrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            maph_topic_name_, 10);
        
        // Subscriber
        sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic_, 10,
            std::bind(&PointCloudToGridNode::pointcloudCallback, this, std::placeholders::_1));
        
        // Parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PointCloudToGridNode::parametersCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PointCloud to Grid node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", cloud_in_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing intensity grid to: %s", mapi_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing height grid to: %s", maph_topic_name_.c_str());
    }

private:
    void updateParameters()
    {
        cell_size_ = this->get_parameter("cell_size").as_double();
        position_x_ = this->get_parameter("position_x").as_double();
        position_y_ = this->get_parameter("position_y").as_double();
        length_x_ = this->get_parameter("length_x").as_double();
        length_y_ = this->get_parameter("length_y").as_double();
        intensity_factor_ = this->get_parameter("intensity_factor").as_double();
        height_factor_ = this->get_parameter("height_factor").as_double();
        cloud_in_topic_ = this->get_parameter("cloud_in_topic").as_string();
        mapi_topic_name_ = this->get_parameter("mapi_topic_name").as_string();
        maph_topic_name_ = this->get_parameter("maph_topic_name").as_string();
        
        // Calculate grid parameters
        topleft_x_ = position_x_ + length_x_ / 2.0;
        topleft_y_ = position_y_ + length_y_ / 2.0;
        bottomright_x_ = position_x_ - length_x_ / 2.0;
        bottomright_y_ = position_y_ - length_y_ / 2.0;
        cell_num_x_ = static_cast<int>(length_x_ / cell_size_);
        cell_num_y_ = static_cast<int>(length_y_ / cell_size_);
        
        RCLCPP_INFO(this->get_logger(), "Grid size: %d x %d cells", cell_num_x_, cell_num_y_);
    }
    
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto & param : parameters) {
            RCLCPP_INFO(this->get_logger(), "Parameter changed: %s", param.get_name().c_str());
        }
        
        updateParameters();
        return result;
    }
    
    PointXY getIndex(double x, double y)
    {
        PointXY ret;
        ret.x = static_cast<int>(std::fabs(x - topleft_x_) / cell_size_);
        ret.y = static_cast<int>(std::fabs(y - topleft_y_) / cell_size_);
        return ret;
    }
    
    void initGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        grid->info.resolution = cell_size_;
        grid->info.width = cell_num_x_;
        grid->info.height = cell_num_y_;
        grid->info.origin.position.x = bottomright_x_;
        grid->info.origin.position.y = bottomright_y_;
        grid->info.origin.position.z = 0.0;
        grid->info.origin.orientation.w = 1.0;
        grid->data.resize(cell_num_x_ * cell_num_y_, 0);
    }
    
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Initialize grids
        auto intensity_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        auto height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        
        initGrid(intensity_grid);
        initGrid(height_grid);
        
        // Initialize grid vectors
        std::vector<int8_t> hpoints(cell_num_x_ * cell_num_y_, 0);
        std::vector<int8_t> ipoints(cell_num_x_ * cell_num_y_, 0);
        
        // Process point cloud
        for (const auto& point : cloud->points)
        {
            if (std::fabs(point.x) > 0.01)
            {
                if (point.x > bottomright_x_ && point.x < topleft_x_)
                {
                    if (point.y > bottomright_y_ && point.y < topleft_y_)
                    {
                        PointXY cell = getIndex(point.x, point.y);
                        
                        if (cell.x < cell_num_x_ && cell.y < cell_num_y_)
                        {
                            int index = (cell_num_x_ * cell_num_y_) - 
                                       (cell.y * cell_num_x_ + cell.x);
                            
                            ipoints[index] = static_cast<int8_t>(
                                std::min(127.0, point.intensity * intensity_factor_));
                            hpoints[index] = static_cast<int8_t>(
                                std::min(127.0, point.z * height_factor_));
                        }
                        else
                        {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cell out of range: %d - %d ||| %d - %d", 
                                cell.x, cell_num_x_, cell.y, cell_num_y_);
                        }
                    }
                }
            }
        }
        
        // Publish intensity grid
        intensity_grid->header.stamp = this->now();
        intensity_grid->header.frame_id = msg->header.frame_id;
        intensity_grid->info.map_load_time = this->now();
        intensity_grid->data = ipoints;
        pub_igrid_->publish(*intensity_grid);
        
        // Publish height grid
        height_grid->header.stamp = this->now();
        height_grid->header.frame_id = msg->header.frame_id;
        height_grid->info.map_load_time = this->now();
        height_grid->data = hpoints;
        pub_hgrid_->publish(*height_grid);
    }
    
    // Parameters
    double cell_size_;
    double position_x_;
    double position_y_;
    double length_x_;
    double length_y_;
    double intensity_factor_;
    double height_factor_;
    std::string cloud_in_topic_;
    std::string mapi_topic_name_;
    std::string maph_topic_name_;
    
    // Calculated parameters
    double topleft_x_;
    double topleft_y_;
    double bottomright_x_;
    double bottomright_y_;
    int cell_num_x_;
    int cell_num_y_;
    
    // ROS 2 interfaces
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_igrid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_hgrid_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToGridNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
