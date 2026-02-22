#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

// Perbaikan Include: Gunakan huruf kecil semua untuk nama file header
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"



// Sesuaikan dengan nama paket baru 'localization'
#include <localization/marvelmind_ros2.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace Eigen;
using namespace std;

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

#define r_robot_gps 0.0
#define offset_gpsx 0.0
#define offset_gpsy 0.0
#define gps_f_sample 10.0 // Sesuaikan nilainya

// Global Variables (Saran: Sebaiknya dimasukkan ke dalam class member di riset berikutnya)
float odox_buf, odoy_buf, odow_buf, odow_buf_prev;
float odox_offset, odoy_offset, odow_offset;
float gpsx_buf, gpsy_buf, gpsw_buf, gpsw_rad_buf;
float gpsx, gpsy, gpsw, gpsw_rad;
float gpsx_prev, gpsy_prev, gpsw_prev, gpsw_rad_prev;
float gps_quality, gps_q;
int status_init_pos;
float robotx, roboty, robotw;
float vx_lokal, vy_lokal, vw_lokal, pos_mpu_rad, vw_mpu;
float vx_global, vy_global, vw_global;
float vx_gps, vy_gps, vw_gps;
float w_mpu, theta_cmps, theta_mpu;
float yaw, yaw_degrees;
float delta_t = 0.01;

// Matriks Kalman (Tetap menggunakan Eigen)
Matrix <float, 3, 1> Xk;
Matrix <float, 3, 3> A = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 1> Xk_prev; 
Matrix <float, 3, 1> Xk_cur; 
Matrix <float, 3, 1> Vt = Matrix<float, 3, 1>::Zero();
Matrix <float, 3, 3> Pk = Matrix<float, 3, 3>::Identity() * 0.1;
Matrix <float, 3, 3> Pk_prev = Matrix<float, 3, 3>::Identity() * 0.1;
Matrix <float, 3, 3> Pk_cur = Matrix<float, 3, 3>::Identity() * 0.1;
Matrix <float, 3, 3> Qk = Matrix<float, 3, 3>::Identity() * pow(0.001, 2);
Matrix <float, 3, 3> I = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 1> Yk = Matrix<float, 3, 1>::Zero();
Matrix <float, 3, 1> Zk;
Matrix <float, 3, 1> Wt = Matrix<float, 3, 1>::Zero();
Matrix <float, 3, 3> Hk = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 3> Sk;
Matrix <float, 3, 3> Rk = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 3> Kk;

class fusion_ekf : public rclcpp::Node 
{
    // Perbaikan: Class Name untuk Message dimulai dengan Huruf Kapital
    rclcpp::Subscription<udp_bot_msgs::msg::TerimaUdp>::SharedPtr sub_robot_pos;
    rclcpp::Subscription<marvelmind_msgs::msg::HedgePosAng>::SharedPtr gps_pos_sub;
    rclcpp::Subscription<marvelmind_msgs::msg::HedgeQuality>::SharedPtr gps_quality_sub;
    rclcpp::Subscription<marvelmind_msgs::msg::HedgeImuFusion>::SharedPtr imu_fusion_sub;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_pos;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_gps;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_odo;
    rclcpp::Publisher<udp_bot_msgs::msg::KirimKecepatanUdp>::SharedPtr pub_robot_vel;
    rclcpp::Publisher<udp_bot_msgs::msg::KirimOffsetUdp>::SharedPtr pub_robot_offset;

    rclcpp::TimerBase::SharedPtr simple_nav_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    udp_bot_msgs::msg::KirimKecepatanUdp msg;
    udp_bot_msgs::msg::KirimOffsetUdp offset_msg;

public:
    fusion_ekf() : Node("fusion_ekf")
    {
        // Subscriber initialization
        sub_robot_pos = this->create_subscription<udp_bot_msgs::msg::TerimaUdp>(
            "data_terima_udp", 10, std::bind(&fusion_ekf::robot_odom_callback, this, std::placeholders::_1));

        gps_pos_sub = this->create_subscription<marvelmind_msgs::msg::HedgePosAng>(
            "hedgehog_pos_ang", 20, std::bind(&fusion_ekf::robot_hedge_callback, this, std::placeholders::_1));

        gps_quality_sub = this->create_subscription<marvelmind_msgs::msg::HedgeQuality>(
            "hedgehog_quality", 20, std::bind(&fusion_ekf::hedge_quality_callback, this, std::placeholders::_1));

        imu_fusion_sub = this->create_subscription<marvelmind_msgs::msg::HedgeImuFusion>(
            "hedgehog_imu_fusion", 20, std::bind(&fusion_ekf::hedge_imu_fus_callback, this, std::placeholders::_1));
 
        // Publisher initialization
        pub_robot_offset = this->create_publisher<udp_bot_msgs::msg::KirimOffsetUdp>("offset_kirim_udp", 10);
        pub_robot_vel = this->create_publisher<udp_bot_msgs::msg::KirimKecepatanUdp>("kecepatan_kirim_udp", 10);
        pub_robot_pos = this->create_publisher<geometry_msgs::msg::Pose>("robot_position", 10);
        pub_robot_gps = this->create_publisher<geometry_msgs::msg::Pose>("robot_gps_position", 10);
        pub_robot_odo = this->create_publisher<geometry_msgs::msg::Pose>("robot_odometry", 10);

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        simple_nav_timer = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&fusion_ekf::fusion_process_callback, this)); 
    }

    // Callback functions
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
        vx_global = msg->vx_global;
        vy_global = msg->vy_global;
        vw_global = msg->vw_global;
    }

    void hedge_imu_fus_callback(const marvelmind_msgs::msg::HedgeImuFusion::SharedPtr msg )
    {
        float qw = (float)msg->qw;
        float qx = (float)msg->qx;
        float qy = (float)msg->qy;
        float qz = (float)msg->qz;
        yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
        yaw_degrees = yaw * TO_DEG;
    }

    void robot_hedge_callback(const marvelmind_msgs::msg::HedgePosAng::SharedPtr msg )
    {
        gpsw_buf = yaw_degrees;
        gpsw_rad_buf = yaw;
        gpsx_buf = (float)msg->x_m - (cosf(gpsw_buf*TO_RAD)*r_robot_gps) - offset_gpsx;
        gpsy_buf = (float)msg->y_m - (sinf(gpsw_buf*TO_RAD)*r_robot_gps) - offset_gpsy;
        vx_gps = (gpsx_buf - gpsx_prev)*gps_f_sample;
        vy_gps = (gpsy_buf - gpsy_prev)*gps_f_sample;
        float vw_temp = gpsw_buf - gpsw_prev;
        while (vw_temp > 180) vw_temp -= 360;
        while (vw_temp < -180) vw_temp += 360;
        vw_gps = vw_temp*TO_RAD*gps_f_sample;
        gpsx_prev = gpsx_buf;
        gpsy_prev = gpsy_buf;
        gpsw_prev = gpsw_buf; 
        gpsw_rad_prev =  gpsw_rad_buf;
    }

    void hedge_quality_callback(const marvelmind_msgs::msg::HedgeQuality::SharedPtr msg)
    {
        gps_quality = (float)(msg->quality_percents)/100;
    }

    void fusion_process_callback()
    {
        // Logika Kalman Filter (Sesuai dengan kode asli kamu)
        gpsx = gpsx_buf;
        gpsy = gpsy_buf;
        gpsw = theta_cmps;
        gpsw_rad = gpsw_rad_buf;
        gps_q = gps_quality;

        if(status_init_pos == 0)
        {
            if(gps_q > 0.95)
            {
                odox_offset = odox_buf - gpsx; 
                odoy_offset = odoy_buf - gpsy;
                odow_offset = odow_buf - gpsw;
                offset_msg.posisi_x_offset = odox_offset;
                offset_msg.posisi_y_offset = odoy_offset;
                offset_msg.sudut_w_offset  = odow_offset;
                pub_robot_offset->publish(offset_msg);
                robotx = odox_buf - odox_offset;
                roboty = odoy_buf - odoy_offset;
                robotw = odow_buf - odow_offset;
                while (robotw > 180) robotw -= 360;
                while (robotw < -180) robotw += 360;
                RCLCPP_INFO_STREAM(get_logger(), "INIT DONE! X: " << robotx << " Y: " << roboty << " W: " << robotw);
                status_init_pos = 1;
                Xk_prev << robotx, roboty, robotw * TO_RAD;
            } 
        }
        else 
        {   
            // Prediksi State
            Matrix <float, 3, 1> BUk;
            BUk(0) = Xk_prev(0) + (vx_lokal * delta_t) * cosf(Xk_prev(2) + (w_mpu * delta_t)/2);
            BUk(1) = Xk_prev(1) + (vx_lokal * delta_t) * sinf(Xk_prev(2) + (w_mpu * delta_t)/2);
            BUk(2) = Xk_prev(2) + (w_mpu * delta_t);
            Xk = BUk + Vt;

            while (Xk(2) > M_PI) Xk(2) -= 2 * M_PI;
            while (Xk(2) < -M_PI) Xk(2) += 2 * M_PI;

            // Jacobian Fk
            Matrix <float, 3, 3> Fk;
            Fk << 1.0, 0.0, -(vx_lokal * delta_t) * sinf(Xk_prev(2) + (w_mpu * delta_t)/2),
                  0.0, 1.0, (vx_lokal * delta_t) * cosf(Xk_prev(2) + (w_mpu * delta_t)/2),
                  0.0, 0.0, 1.0;

            Pk = Fk * Pk_prev * Fk.transpose() + Qk;

            // Koreksi (Measurement)
            Zk << gpsx, gpsy, gpsw;
            Yk = Zk - Xk + Wt;
            while(Yk(2) > M_PI) Yk(2) -= 2 * M_PI;
            while(Yk(2) < -M_PI) Yk(2) += 2 * M_PI;

            Sk = Hk * Pk * Hk.transpose() + Rk;
            Kk = Pk * Hk.transpose() * Sk.inverse();

            Xk_cur = Xk + Kk * Yk;
            while (Xk_cur(2) > M_PI) Xk_cur(2) -= 2 * M_PI;
            while (Xk_cur(2) < -M_PI) Xk_cur(2) += 2 * M_PI;

            Pk_cur = Pk - Kk * Hk * Pk;
            Xk_prev = Xk_cur;
            Pk_prev = Pk_cur;

            robotx = Xk_cur(0);
            roboty = Xk_cur(1);
            robotw = Xk_cur(2) * TO_DEG;

            while (robotw > 180) robotw -= 360;
            while (robotw < -180) robotw += 360;
            
            // Offset Update
            odox_offset = odox_buf - robotx;
            odoy_offset = odoy_buf - roboty;
            odow_offset = odow_buf - robotw;
            offset_msg.posisi_x_offset = odox_offset;
            offset_msg.posisi_y_offset = odoy_offset;
            offset_msg.sudut_w_offset  = odow_offset;
            pub_robot_offset->publish(offset_msg);
        }

        // Publish Pose & TF
        geometry_msgs::msg::Pose pos_msg;
        pos_msg.position.x = robotx;
        pos_msg.position.y = roboty;
        tf2::Quaternion q;
        q.setRPY(0, 0, robotw * TO_RAD);
        pos_msg.orientation = tf2::toMsg(q);
        pub_robot_pos->publish(pos_msg);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = robotx;
        t.transform.translation.y = roboty;
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster->sendTransform(t);
    }
};

// Perbaikan fungsi main: Hindari pembuatan node ganda
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fusion_ekf>();
    RCLCPP_INFO(node->get_logger(), "START FUSION EKF NODE !!!!");
    
    // Tunggu sebentar agar inisialisasi sensor stabil
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}