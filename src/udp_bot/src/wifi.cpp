#include "rclcpp/rclcpp.hpp"

// PERBAIKAN: Menggunakan udp_bot_msgs sesuai permintaan
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <cstring>
#include <chrono>
#include <memory>
// PERBAIKAN: Sesuaikan path include dengan folder Anda
#include "udp_bot/UdpSocket.h" 
#include <functional>

using namespace std::chrono_literals;

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252
float delta_t = 0.01;

////untuk ip tujuan kirim data dan portnya Wifi
const char *addr_lan = "192.168.0.162";
uint16_t port_lan = 8787;
////untuk bind dan local port Wifi
const char *listenAddr_lan = "0.0.0.0";
uint16_t localPort_lan = 9999;

class UnicastLan : public rclcpp::Node {
public:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_robot_pos_geo;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_robot_gps;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_robot_odo;
    rclcpp::TimerBase::SharedPtr pos_send_timer;

    char lan_kirim[40] = {'t','e','u'};
    float robot_posx, robot_posy, robot_posw;
    float robot_gpsx, robot_gpsy, robot_gpsw;
    float robot_odox, robot_odoy, robot_odow;
    double roll, pitch, yaw;
    double roll_gps, pitch_gps, yaw_gps;
    double roll_odo, pitch_odo, yaw_odo;

    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastLan> m_unicast;

    // PERBAIKAN: Constructor definisi langsung di dalam class (menghapus kualifikasi UnicastLan::)
    UnicastLan(const std::string &node_name_lan, const char *remoteAddr_lan, const char *listenAddr_lan, uint16_t localPort_lan, uint16_t port_lan)
    : Node(node_name_lan),
      m_socketOpts({sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr_lan}),
      m_unicast(*this, &m_socketOpts) 
    {
        sub_robot_pos_geo = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot_position", 10, std::bind(&UnicastLan::geo_robot_pos_callback, this, std::placeholders::_1));
        sub_robot_gps = create_subscription<geometry_msgs::msg::Pose>(
            "robot_gps_position", 10, std::bind(&UnicastLan::robot_gps_callback, this, std::placeholders::_1));
        sub_robot_odo = create_subscription<geometry_msgs::msg::Pose>(
            "robot_odometry", 10, std::bind(&UnicastLan::robot_odo_callback, this, std::placeholders::_1));

        sockets::SocketRet ret = m_unicast.startUnicast(remoteAddr_lan, localPort_lan, port_lan);

        if (ret.m_success) {
            RCLCPP_INFO(get_logger(), "Listening on WIFI %s:%d sending to %s:%d", listenAddr_lan, localPort_lan, remoteAddr_lan, port_lan);
        } else {
            RCLCPP_ERROR(get_logger(), "Error: %s", ret.m_msg.c_str());
            rclcpp::shutdown();
            return;
        }

        pos_send_timer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&UnicastLan::send_pos_data_callback, this));
    }

    void sendMsg(const char *data, size_t len) {
        auto ret = m_unicast.sendMsg(data, len);
        if (!ret.m_success) {
            RCLCPP_ERROR(get_logger(), "Send Error: %s", ret.m_msg.c_str());
        }
    }

    void onReceiveData(const char *data, size_t size) {
        std::string str(reinterpret_cast<const char *>(data), size);
        std::cout << "Received: " << str << "\n";
    }

    void geo_robot_pos_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        robot_posx = (float)msg->position.x;
        robot_posy = (float)msg->position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll,pitch,yaw);
        robot_posw = (float)yaw*TO_DEG;
    }

    void robot_gps_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        robot_gpsx = msg->position.x;
        robot_gpsy = msg->position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_gps, pitch_gps, yaw_gps);
        robot_gpsw = yaw_gps * TO_DEG;
    }

    void robot_odo_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        robot_odox = msg->position.x;
        robot_odoy = msg->position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_odo, pitch_odo, yaw_odo);
        robot_odow = yaw_odo * TO_DEG;
    }

    void send_pos_data_callback() {
        std::memcpy(lan_kirim + 3, &robot_posx, 4);
        std::memcpy(lan_kirim + 7, &robot_posy, 4);
        std::memcpy(lan_kirim + 11, &robot_posw, 4);
        std::memcpy(lan_kirim + 15, &robot_gpsx, 4);
        std::memcpy(lan_kirim + 19, &robot_gpsy, 4);
        std::memcpy(lan_kirim + 23, &robot_gpsw, 4);
        std::memcpy(lan_kirim + 27, &robot_odox, 4);
        std::memcpy(lan_kirim + 31, &robot_odoy, 4);
        std::memcpy(lan_kirim + 35, &robot_odow, 4);
        // PERBAIKAN: Panggil sendMsg milik objek ini, bukan static
        this->sendMsg(lan_kirim, sizeof(lan_kirim));
    }
};
   
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto lan = std::make_shared<UnicastLan>("unicast_lan", addr_lan, listenAddr_lan, localPort_lan, port_lan);
    rclcpp::spin(lan);
    rclcpp::shutdown();
    return 0;
}
