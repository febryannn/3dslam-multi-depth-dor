#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <geometry_msgs/msg/pose.hpp>

// Menggunakan M_PI dari math.h untuk presisi lebih baik
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

using namespace std::chrono_literals;

// Global variables sesuai style kamu
float yaw_q, yaw_deg;
float robotx, roboty, robotw;

class Slamlistnode : public rclcpp::Node
{
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_slam;

    rclcpp::TimerBase::SharedPtr timer;

public:
    Slamlistnode()
    : Node("slam_list_node")
    {
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Pastikan topik ini tidak bentrok dengan node fusion/ekf jika dijalankan bersamaan
        pub_robot_slam = this->create_publisher<geometry_msgs::msg::Pose>(
            "robot_position_slam", 10
        );

        timer = this->create_wall_timer(
            10ms, 
            std::bind(&Slamlistnode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            // Mengambil transformasi dari frame 'map' ke 'base_link'
            t = tf_buffer->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            // Menggunakan THROTTLE agar log tidak memenuhi layar
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "TF transform unavailable: %s", ex.what());
            return;
        }

        // Simpan data posisi
        robotx = t.transform.translation.x;
        roboty = t.transform.translation.y;

        // Konversi Quaternion ke Yaw (Z-axis rotation)
        float qw = t.transform.rotation.w;
        float qx = t.transform.rotation.x;
        float qy = t.transform.rotation.y;
        float qz = t.transform.rotation.z;

        yaw_q = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
        yaw_deg = yaw_q * TO_DEG;
        robotw = yaw_deg;

        // Log hasil ke terminal
        RCLCPP_INFO_STREAM(this->get_logger(), "SLAM Pos -> X: " << robotx << " Y: " << roboty << " Yaw: " << robotw);

        // Publish ke topik Pose
        geometry_msgs::msg::Pose pos_msg;
        pos_msg.position.x = robotx;
        pos_msg.position.y = roboty;
        pos_msg.position.z = 0.0; 

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_q);
        pos_msg.orientation = tf2::toMsg(q);
        pub_robot_slam->publish(pos_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Slamlistnode>();
    RCLCPP_INFO(node->get_logger(), "START SLAM LISTENER NODE !!!!");

    // Memberikan waktu agar TF buffer terisi
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}