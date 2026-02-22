#include "rclcpp/rclcpp.hpp"

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
#include "udp_bot/UdpSocket.h"
#include <functional>

using namespace std::chrono_literals;

#define TO_DEG 57.295779513
float delta_t = 0.01;

const char *addr = "169.254.1.15";
uint16_t port = 4444;

const char *listenAddr = "0.0.0.0";
uint16_t localPort = 8888;

class UnicastApp : public rclcpp::Node {
public:

    udp_bot_msgs::msg::TerimaUdp msg;

    char stm_kirim[64] = {'t','e','l'};
    char stm_terima[64];

    float posisi_x_buffer, posisi_y_buffer, sudut_w_buffer;
    float kecepatan_robotx, kecepatan_robotw;
    int cnt_send_kecepatan = 0;
    float sudut_servo_act;
    uint8_t tombol;
    float vx_global, vy_global, vw_global;

    float kecepatan_x, kecepatan_y, kecepatan_sudut;
    float posisi_x_offset, posisi_y_offset, sudut_w_offset;
    float sudut_servo;

    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastApp> m_unicast;

    rclcpp::Publisher<udp_bot_msgs::msg::TerimaUdp>::SharedPtr terima_udp_pub;
    rclcpp::Subscription<udp_bot_msgs::msg::KirimKecepatanUdp>::SharedPtr kirim_kecepatan_udp_sub;
    rclcpp::Subscription<udp_bot_msgs::msg::KirimOffsetUdp>::SharedPtr kirim_offset_udp_sub;
    rclcpp::TimerBase::SharedPtr udp_send_timer;

    UnicastApp(const std::string &node_name, const char *remoteAddr,
               const char *listenAddr, uint16_t localPort, uint16_t port)
    : Node(node_name),
      m_socketOpts({ sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr}),
      m_unicast(*this, &m_socketOpts)
    {
        terima_udp_pub =
            this->create_publisher<udp_bot_msgs::msg::TerimaUdp>(
                "data_terima_udp", 10);

        kirim_kecepatan_udp_sub =
            this->create_subscription<udp_bot_msgs::msg::KirimKecepatanUdp>(
                "kecepatan_kirim_udp", 10,
                std::bind(&UnicastApp::udp_kecepatan_send_callback,
                          this, std::placeholders::_1));

        kirim_offset_udp_sub =
            this->create_subscription<udp_bot_msgs::msg::KirimOffsetUdp>(
                "offset_kirim_udp", 10,
                std::bind(&UnicastApp::udp_offset_send_callback,
                          this, std::placeholders::_1));

        auto ret = m_unicast.startUnicast(remoteAddr, localPort, port);
        if (!ret.m_success) {
            RCLCPP_ERROR(this->get_logger(), "UDP error: %s", ret.m_msg.c_str());
            rclcpp::shutdown();
        }

        udp_send_timer = this->create_wall_timer(
            10ms,
            std::bind(&UnicastApp::send_udp_data_callback, this));
    }

    void onReceiveData(const char *data, size_t size)
    {
        std::memcpy(stm_terima, data, size);

        std::memcpy(&posisi_x_buffer,  stm_terima + 3,  4);
        std::memcpy(&posisi_y_buffer,  stm_terima + 7,  4);
        std::memcpy(&sudut_w_buffer,   stm_terima + 11, 4);
        std::memcpy(&kecepatan_robotx, stm_terima + 15, 4);
        std::memcpy(&kecepatan_robotw, stm_terima + 23, 4);
        std::memcpy(&sudut_servo_act,  stm_terima + 27, 4);
        std::memcpy(&tombol,           stm_terima + 31, 1);
        std::memcpy(&vx_global,        stm_terima + 32, 4);
        std::memcpy(&vy_global,        stm_terima + 36, 4);
        std::memcpy(&vw_global,        stm_terima + 40, 4);

        msg.posisi_x_buffer  = posisi_x_buffer;
        msg.posisi_y_buffer  = posisi_y_buffer;
        msg.sudut_w_buffer   = sudut_w_buffer;
        msg.kecepatan_robotx = kecepatan_robotx;
        msg.kecepatan_robotw = kecepatan_robotw;
        msg.sudut_servo_act  = sudut_servo_act;
        msg.tombol           = tombol;
        msg.vx_global        = vx_global;
        msg.vy_global        = vy_global;
        msg.vw_global        = vw_global;

        terima_udp_pub->publish(msg);
    }

    void sendMsg(const char *data, size_t len)
    {
        auto ret = m_unicast.sendMsg(data, len);
        if (!ret.m_success)
            RCLCPP_ERROR(this->get_logger(), "Send Error");
    }

    void udp_kecepatan_send_callback(
        const udp_bot_msgs::msg::KirimKecepatanUdp::SharedPtr msg)
    {
        kecepatan_x      = msg->kecepatan_x;
        kecepatan_y      = msg->kecepatan_y;
        kecepatan_sudut  = msg->kecepatan_sudut;
        cnt_send_kecepatan = 0;
    }

    void udp_offset_send_callback(
        const udp_bot_msgs::msg::KirimOffsetUdp::SharedPtr msg)
    {
        posisi_x_offset = msg->posisi_x_offset;
        posisi_y_offset = msg->posisi_y_offset;
        sudut_w_offset  = msg->sudut_w_offset;
    }

    void send_udp_data_callback()
    {
        if (cnt_send_kecepatan >= 100) {
            kecepatan_x     = 0;
            kecepatan_y     = 0;
            kecepatan_sudut = 0;
        }

        if (cnt_send_kecepatan < 100)
            cnt_send_kecepatan++;

        std::memcpy(stm_kirim + 3,  &kecepatan_y,      4);
        std::memcpy(stm_kirim + 7,  &kecepatan_x,      4);
        std::memcpy(stm_kirim + 11, &kecepatan_sudut,  4);
        std::memcpy(stm_kirim + 15, &posisi_x_offset,  4);
        std::memcpy(stm_kirim + 19, &posisi_y_offset,  4);
        std::memcpy(stm_kirim + 23, &sudut_w_offset,   4);
        std::memcpy(stm_kirim + 27, &sudut_servo,      4);

        sendMsg(stm_kirim, sizeof(stm_kirim));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto app = std::make_shared<UnicastApp>("UDP_com", addr, listenAddr, localPort, port);
    rclcpp::spin(app);
    rclcpp::shutdown();
    return 0;
}
