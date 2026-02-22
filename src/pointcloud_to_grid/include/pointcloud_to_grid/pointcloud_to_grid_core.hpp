#ifndef POINTCLOUD_TO_GRID_CORE_HPP_
#define POINTCLOUD_TO_GRID_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>

// Struktur data untuk koordinat sel
class PointXY {
public: 
    int x;
    int y;
};

// Struktur data untuk titik awan poin (PointXYZI)
class PointXYZI {
public: 
    double x;
    double y;
    double z;
    double intensity;
};

class GridMap {
public: 
    float position_x;
    float position_y;
    float cell_size;
    float length_x;
    float length_y;
    std::string cloud_in_topic;
    std::string frame_out;
    std::string mapi_topic_name;
    std::string maph_topic_name;
    float topleft_x;
    float topleft_y;
    float bottomright_x;
    float bottomright_y;
    int cell_num_x;
    int cell_num_y;
    float intensity_factor;
    float height_factor;

    // Inisialisasi Grid menggunakan format ROS 2
    void initGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
        grid->header.frame_id = this->frame_out;
        grid->info.origin.position.z = 0.0;
        grid->info.origin.orientation.w = 1.0;
        grid->info.origin.orientation.x = 0.0;
        grid->info.origin.orientation.y = 0.0;
        grid->info.origin.orientation.z = 0.0;
        grid->info.origin.position.x = position_x - length_x / 2.0;
        grid->info.origin.position.y = position_y - length_y / 2.0;
        grid->info.width = static_cast<uint32_t>(length_x / cell_size);
        grid->info.height = static_cast<uint32_t>(length_y / cell_size);
        grid->info.resolution = cell_size;
    }
    
    // Refresh parameter grid
    void paramRefresh(rclcpp::Logger logger) {
        topleft_x = position_x + length_x / 2.0;
        bottomright_x = position_x - length_x / 2.0;
        topleft_y = position_y + length_y / 2.0;
        bottomright_y = position_y - length_y / 2.0;
        cell_num_x = static_cast<int>(length_x / cell_size);
        cell_num_y = static_cast<int>(length_y / cell_size);
        
        if(cell_num_x > 0) {
            RCLCPP_INFO(logger, "Grid Config: %d*%d px, Subscribed to %s", 
                        cell_num_x, cell_num_y, cloud_in_topic.c_str());
        }
    }

    int getSize() { return cell_num_x * cell_num_y; }
    int getSizeX() { return cell_num_x; }
    int getSizeY() { return cell_num_y; }
    double getLengthX() { return length_x; }
    double getLengthY() { return length_y; }
    double getResolution() { return cell_size; }
};

#endif
