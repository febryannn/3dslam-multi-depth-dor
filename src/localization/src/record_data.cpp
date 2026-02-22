#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

// Perbaikan: Header menggunakan huruf kecil semua dan melengkapi baris yang kosong
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"

// Perbaikan: Sesuaikan dengan nama paket 'localization'
#include <localization/marvelmind_ros2.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <filesystem>

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

// Global Variables
float odox_buf, odoy_buf, odow_buf, odow_buf_prev;
float gpsx_buf, gpsy_buf, gpsw_buf, gpsw_rad_buf;
float vx_lokal, vy_lokal, vw_lokal, pos_mpu_rad, v_prev;
float vx_global, vy_global, vw_global;
float vx_gps, vy_gps, vw_gps;
float w_mpu, theta_cmps, theta_mpu, theta, degree_cmps, temp_cmps;

class DataLogger : public rclcpp::Node {
public:
    // Perbaikan: Nama tipe data pesan menggunakan Huruf Kapital di awal (TerimaUdp)
    rclcpp::Subscription<udp_bot_msgs::msg::TerimaUdp>::SharedPtr sub_robot_pos;
rclcpp::Subscription<marvelmind_msgs::msg::HedgePosAng>::SharedPtr gps_pos_sub;
    rclcpp::TimerBase::SharedPtr timer;

    DataLogger() : Node("DataLogger") {
        // Membuat direktori log di home user
        std::string log_dir = "/home/" + std::string(std::getenv("USER")) + "/agv2_ws" + "/ros2_record_tristan";
        std::filesystem::create_directories(log_dir);
        file_.open(log_dir + "/cmps_test2.csv", std::ios::out);
        
        // Menulis header CSV
        file_ << "time,vx_lokal,w_mpu,gps_x,gps_y,theta_cmps,vw_lokal,theta_mpu,degree_cmps\n";
        
        // Inisialisasi Subscriber
        sub_robot_pos = this->create_subscription<udp_bot_msgs::msg::TerimaUdp>(
            "data_terima_udp", 10,
            std::bind(&DataLogger::robot_odom_callback, this, std::placeholders::_1));

        gps_pos_sub = this->create_subscription<marvelmind_msgs::msg::HedgePosAng>(
            "hedgehog_pos_ang", 20,
            std::bind(&DataLogger::robot_hedge_callback, this, std::placeholders::_1));
        
        // Timer untuk pencatatan setiap 10ms
        timer = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&DataLogger::recording, this));
    }

    ~DataLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void robot_odom_callback(const udp_bot_msgs::msg::TerimaUdp::SharedPtr msg)
    {
        odox_buf = msg->posisi_x_buffer;
        odoy_buf = msg->posisi_y_buffer;
        odow_buf = msg->sudut_w_buffer;
        vx_lokal = msg->kecepatan_robotx;
        vw_lokal = msg->kecepatan_robotw;
        w_mpu = msg->w_mpu;
        theta_mpu = msg->rad_w_buffer_mpu;
        theta_cmps = msg->rad_w_buffer_magnet;
        degree_cmps = msg->abs_cmps;

        temp_cmps = theta_cmps * TO_DEG;

        // Normalisasi sudut
        while (theta_cmps > M_PI) theta_cmps -= 2 * M_PI;
        while (theta_cmps < -M_PI) theta_cmps += 2 * M_PI;

        while (temp_cmps > 180) temp_cmps -= 360;
        while (temp_cmps < -180) temp_cmps -= 360;
        
        vx_global = msg->vx_global;
        vy_global = msg->vy_global;
        vw_global = msg->vw_global;
    }

    void robot_hedge_callback(const marvelmind_msgs::msg::HedgePosAng::SharedPtr msg )
    {
        gpsx_buf = (float)msg->x_m; 
        gpsy_buf = (float)msg->y_m; 
    }

    void recording (){
        // Ambil waktu sistem dalam detik
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        double time = now_ms / 1000.0; 

        // Menulis data ke CSV
        file_ << time << "," << vx_lokal << "," << w_mpu << "," 
              << gpsx_buf << "," << gpsy_buf << "," << temp_cmps  << "," 
              << vw_lokal << "," << theta_mpu << "," << degree_cmps << "\n";

        RCLCPP_INFO_STREAM(get_logger(), "Recording: " << time << ", X: " << gpsx_buf << ", Y: " << gpsy_buf);
    }

    std::ofstream file_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Memberikan waktu jeda inisialisasi
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Perbaikan: Hanya buat satu instance node dan spin node tersebut
    auto node = std::make_shared<DataLogger>();
    RCLCPP_INFO(node->get_logger(), "START DATA LOGGING !!!!");

    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "RECORDING FINISHED");
    rclcpp::shutdown();
    return 0;
}