#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

// Perbaikan: Header menggunakan huruf kecil semua
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"

// Perbaikan: Sesuaikan dengan nama paket 'localization'
#include <localization/marvelmind_ros2.hpp>
#include "localization/fusion_gps_odom.h"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

// Global variables (Saran: Sebaiknya dipindahkan ke dalam class member)
float odox_buf, odoy_buf, odow_buf, odow_buf_prev, vw_mpu;
float odox_offset, odoy_offset, odow_offset;
int status_init_pos;
float robotx, roboty, robotw;
float vx_lokal, vy_lokal, vw_lokal;
float vx_global, vy_global, vw_global;
float delta_t = 0.01;

class odom : public rclcpp::Node 
{
    // Perbaikan: Nama tipe data pesan diawali huruf kapital
    rclcpp::Subscription<udp_bot_msgs::msg::TerimaUdp>::SharedPtr sub_robot_pos;
    rclcpp::Publisher<udp_bot_msgs::msg::KirimOffsetUdp>::SharedPtr pub_robot_offset;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_pos;

    rclcpp::TimerBase::SharedPtr simple_nav_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    udp_bot_msgs::msg::KirimKecepatanUdp msg;
    udp_bot_msgs::msg::KirimOffsetUdp offset_msg;

public:
    odom() : Node("odom")
    {
        sub_robot_pos = this->create_subscription<udp_bot_msgs::msg::TerimaUdp>(
            "data_terima_udp", 10,
            std::bind(&odom::robot_odom_callback, this, std::placeholders::_1)
        );

        pub_robot_offset = this->create_publisher<udp_bot_msgs::msg::KirimOffsetUdp>(
            "offset_kirim_udp", 10
        );
        pub_robot_pos = this->create_publisher<geometry_msgs::msg::Pose>(
            "odom_position", 10
        );

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        simple_nav_timer = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&odom::fusion_process_callback, this)); 
    }

    void robot_odom_callback(const udp_bot_msgs::msg::TerimaUdp::SharedPtr msg)
    {
        static double previous_time = 0.0;
        double current_time = this->now().seconds();

        // Hitung delta_t secara dinamis
        if (previous_time != 0.0) {
            delta_t = current_time - previous_time;
        }
        if (delta_t <= 0) delta_t = 0.01; 
        previous_time = current_time;

        odox_buf = msg->posisi_x_buffer;
        odoy_buf = msg->posisi_y_buffer;
        odow_buf = msg->sudut_w_buffer;
        vx_lokal = msg->kecepatan_robotx;
        vw_lokal = msg->kecepatan_robotw;

        // Hitung kecepatan sudut MPU dengan unwrap
        float mpu_temp = odow_buf - odow_buf_prev;
        while (mpu_temp > 180) mpu_temp -= 360;
        while (mpu_temp < -180) mpu_temp += 360;
        vw_mpu = (mpu_temp * TO_RAD) / delta_t;

        vx_global = msg->vx_global;
        vy_global = msg->vy_global;
        vw_global = msg->vw_global;

        odow_buf_prev = odow_buf;
    }

    void fusion_process_callback()
    {
        if(status_init_pos == 0)
        {
            // Inisialisasi parameter awal
            this->declare_parameter<double>("initial_robotx", 0.0);
            this->declare_parameter<double>("initial_roboty", 0.0);
            this->declare_parameter<double>("initial_robotw", 0.0);
            
            this->get_parameter("initial_robotx", robotx);
            this->get_parameter("initial_roboty", roboty);
            this->get_parameter("initial_robotw", robotw);

            odox_offset = odox_buf - robotx;
            odoy_offset = odoy_buf - roboty;
            odow_offset = odow_buf - robotw;
            
            offset_msg.posisi_x_offset = odox_offset;
            offset_msg.posisi_y_offset = odoy_offset;
            offset_msg.sudut_w_offset  = odow_offset;
            pub_robot_offset->publish(offset_msg);
            
            status_init_pos = 1;
            RCLCPP_INFO(this->get_logger(), "[ODOMETRY INITIAL LOCATION DONE]");
        }
        else
        {
            robotx = odox_buf - odox_offset;
            roboty = odoy_buf - odoy_offset;
            robotw = odow_buf - odow_offset;
            
            while (robotw > 180) robotw -= 360;
            while (robotw < -180) robotw += 360;

            geometry_msgs::msg::Pose odom_pose;
            odom_pose.position.x = robotx;
            odom_pose.position.y = roboty;
            tf2::Quaternion q;
            q.setRPY(0, 0, robotw * TO_RAD);
            odom_pose.orientation = tf2::toMsg(q);
            pub_robot_pos->publish(odom_pose);

            // Publish Transform odom -> base_link
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = robotx;
            t.transform.translation.y = roboty;
            t.transform.translation.z = 0.0;
            t.transform.rotation = tf2::toMsg(q);
            tf_broadcaster->sendTransform(t);

            RCLCPP_INFO_STREAM(get_logger(), "pos_X :" << robotx << ", pos_Y :" << roboty << ", pos_w :" << robotw );
        }  
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odom>();
    RCLCPP_INFO(node->get_logger(), "START ODOM NODE !!!!");

    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}