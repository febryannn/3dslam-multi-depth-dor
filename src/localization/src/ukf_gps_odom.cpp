#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

// Perbaikan: Melengkapi include yang kosong dan menggunakan huruf kecil
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"

// Sesuaikan dengan nama paket 'localization'
#include <localization/marvelmind_ros2.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace Eigen;
using namespace std;

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252
#define PI 3.14159265359

#define r_robot_gps 0.0
#define offset_gpsx 1.0
#define offset_gpsy 0.25
#define offset_cmps 90
#define gps_f_sample 0.0

// Global Variables
float odox_buf, odoy_buf, odow_buf, odow_buf_prev;
float odox_offset, odoy_offset, odow_offset;
float gpsx_buf, gpsy_buf, gpsw_buf, gpsw_rad_buf;
float gpsx, gpsy, gpsw, gpsw_rad;
float gpsx_prev, gpsy_prev, gpsw_prev, gpsw_rad_prev;
float gps_quality, gps_q;
int status_init_pos;
float robotx, roboty, robotw;
float vx_lokal, vy_lokal, vw_lokal, pos_mpu_rad, v_prev;
float vx_global, vy_global, vw_global;
float vx_gps, vy_gps, vw_gps;
float w_mpu, theta_cmps, theta_mpu, theta, theta_abs;
float yaw_q, yaw_q_degrees;
bool data_recieved = false;
int marker_counter = 0;

/////// UKF Parameters ///////
float delta_t = 0.01;
float n = 3;
double ALPHA = 0.06;
double BETA = 2.0;
double KAPPA = 0.0;
double LAMBDA;

Matrix<float, 3, 1> Xk_prev; 
Matrix<float, 3, 1> Xk_pred; 
Matrix<float, 3, 1> Xk_cur;  

Matrix <float, 3, 3> Pk_prev = (Matrix <float, 3, 3>() << 
                            pow(0.02,2), 0.0,        0.0,
                            0.0,        pow(0.02,2), 0.0,
                            0.0,        0.0,        pow(1.0,2)).finished();

Matrix<float, 3, 3> Pk_pred, Pk_cur, chol;
Matrix<float, 3, 1> Zk_pred, Zk_actual;

Matrix <float, 3, 3> Qk = Matrix<float, 3, 3>::Identity() * pow(0.001, 2);
Matrix <float, 3, 3> Rk = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 3> Gps_Q = Matrix<float, 3, 3>::Zero();
Matrix <float, 3, 3> H = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 3> I = Matrix<float, 3, 3>::Identity();
Matrix <float, 3, 3> R_adapt = Matrix<float, 3, 3>::Zero();
Matrix <float, 3, 3> Sk, Tk, Kk;
Matrix<float, 3, 7> sig, sig_pred, Zk_sig_pred;
Matrix<float, 7, 1> Wm, Wc;
Matrix<float, 3, 1> dxp, dzp, dx, dz;

class fusion_ukf : public rclcpp::Node 
{
    // Perbaikan: Nama tipe data pesan (Capitalized)
    rclcpp::Subscription<udp_bot_msgs::msg::TerimaUdp>::SharedPtr sub_robot_pos;
    rclcpp::Subscription<marvelmind_msgs::msg::HedgePosAng>::SharedPtr gps_pos_sub;
    rclcpp::Subscription<marvelmind_msgs::msg::HedgeQuality>::SharedPtr gps_quality_sub;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_pos;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_gps;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_odo;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<udp_bot_msgs::msg::KirimOffsetUdp>::SharedPtr pub_robot_offset;

    rclcpp::TimerBase::SharedPtr simple_nav_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    udp_bot_msgs::msg::KirimKecepatanUdp msg;
    udp_bot_msgs::msg::KirimOffsetUdp offset_msg;

public:
    fusion_ukf() : Node("fusion_ukf")
    {
        sub_robot_pos = this->create_subscription<udp_bot_msgs::msg::TerimaUdp>(
            "bot1/data_terima_udp", 10, std::bind(&fusion_ukf::robot_odom_callback, this, std::placeholders::_1));

        gps_pos_sub = this->create_subscription<marvelmind_msgs::msg::HedgePosAng>(
            "bot1/hedgehog_pos_ang", 20, std::bind(&fusion_ukf::robot_hedge_callback, this, std::placeholders::_1));

        gps_quality_sub = this->create_subscription<marvelmind_msgs::msg::HedgeQuality>(
            "bot1/hedgehog_quality", 20, std::bind(&fusion_ukf::hedge_quality_callback, this, std::placeholders::_1));
 
        pub_robot_offset = this->create_publisher<udp_bot_msgs::msg::KirimOffsetUdp>("bot1/offset_kirim_udp", 10);
        pub_robot_pos = this->create_publisher<geometry_msgs::msg::Pose>("bot1/robot_position", 10);
        pub_robot_gps = this->create_publisher<geometry_msgs::msg::Pose>("bot1/robot_gps_position", 10);
        pub_robot_odo = this->create_publisher<geometry_msgs::msg::Pose>("bot1/robot_odometry", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bot1/ukf_sigma_markers", 10);

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        simple_nav_timer = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&fusion_ukf::fusion_process_callback, this)); 
    }

    void robot_odom_callback(const udp_bot_msgs::msg::TerimaUdp::SharedPtr msg)
    {
        odox_buf = msg->posisi_x_buffer;
        odoy_buf = msg->posisi_y_buffer;
        odow_buf = msg->sudut_w_buffer;
        vx_lokal = msg->kecepatan_robotx;
        vw_lokal = msg->kecepatan_robotw;
        w_mpu = msg->w_mpu;
        theta_mpu = msg->rad_w_buffer_mpu;
        theta_cmps = msg->rad_w_buffer_magnet + offset_cmps*TO_RAD;
        theta_abs = msg->abs_cmps;

        while (theta_cmps > M_PI) theta_cmps -= 2 * M_PI;
        while (theta_cmps < -M_PI) theta_cmps += 2 * M_PI;
        
        vx_global = msg->vx_global;
        vy_global = msg->vy_global;
        vw_global = msg->vw_global;
    }

    void robot_hedge_callback(const marvelmind_msgs::msg::HedgePosAng::SharedPtr msg )
    {
        gpsw_buf = yaw_q_degrees;
        gpsw_rad_buf = yaw_q;
        float dx_offset = 0.0f; 
        float dy_offset = 0.0f;  

        gpsx_buf = (float)msg->x_m - (cosf(theta_cmps) * dx_offset - sinf(theta_cmps) * dy_offset) + offset_gpsx;
        gpsy_buf = (float)msg->y_m - (sinf(theta_cmps) * dx_offset + cosf(theta_cmps) * dy_offset) + offset_gpsy;
        data_recieved = true;
    }

    void hedge_quality_callback(const marvelmind_msgs::msg::HedgeQuality::SharedPtr msg)
    {
        gps_quality = (float)(msg->quality_percents)/100;
    }

    void fusion_process_callback()
    {
        gpsx = gpsx_buf;
        gpsy = gpsy_buf;
        gpsw = theta_cmps;
        gps_q = gps_quality;

        if (!data_recieved) return;

        if(status_init_pos == 0)
        {
            if(gps_q > 0.95)
            {
                odox_offset = odox_buf - gpsx; 
                odoy_offset = odoy_buf - gpsy;
                odow_offset = odow_buf - gpsw * TO_DEG;
                offset_msg.posisi_x_offset = odox_offset;
                offset_msg.posisi_y_offset = odoy_offset;
                offset_msg.sudut_w_offset  = odow_offset;
                pub_robot_offset->publish(offset_msg);
                
                robotx = odox_buf - odox_offset;
                roboty = odoy_buf - odoy_offset;
                robotw = odow_buf - odow_offset;
                while (robotw > 180) robotw -= 360;
                while (robotw < -180) robotw += 360;

                status_init_pos = 1;
                Xk_prev << robotx, roboty, robotw * TO_RAD;
            } 
        }
        else 
        {   
            // UKF Logic (Sesuai kode asli kamu)
            LAMBDA = (ALPHA * ALPHA ) * (n + KAPPA) - n;
            Wm.setZero(); Wc.setZero();
            Wm[0] = LAMBDA / (n + LAMBDA);
            Wc[0] = LAMBDA / (n + LAMBDA) + (1 - ALPHA * ALPHA + BETA);
            for (int i = 1; i < 2*n + 1; i++) {
                Wm[i] = 1.0 / (2 * (n + LAMBDA));
                Wc[i] = 1.0 / (2 * (n + LAMBDA));
            }

            chol = ((n + LAMBDA) * Pk_prev).llt().matrixL();
            sig.col(0) = Xk_prev;
            for (int k = 0; k < n; k++) {
                sig.col(k + 1) = Xk_prev + chol.col(k);
                sig.col(k + 1 + n) = Xk_prev - chol.col(k);
            }
        
            sig_pred.setZero();
            for (int i = 0; i < 2*n + 1; i++) {
                theta = sig(2, i);
                sig_pred(0, i) = sig(0, i) + vx_lokal * cosf(theta) * delta_t;
                sig_pred(1, i) = sig(1, i) + vx_lokal * sinf(theta) * delta_t;
                sig_pred(2, i) = sig(2, i) + w_mpu * delta_t;
            }
            Xk_pred = sig_pred * Wm;

            Pk_pred.setZero();
            for (int i = 0; i < 2*n + 1; i++) {
                dxp = sig_pred.col(i) - Xk_pred;
                Pk_pred += Wc(i) * (dxp * dxp.transpose());
            }
            Pk_pred += Qk; 

            Zk_sig_pred = sig_pred;
            Zk_pred = Zk_sig_pred * Wm;
            Sk.setZero();
            for (int i = 0; i < 2*n + 1; i++) {
                dzp =  Zk_sig_pred.col(i) - Zk_pred;
                Sk += Wc(i) * (dzp * dzp.transpose());
            }
            Sk += Rk;

            Tk.setZero();
            for (int i = 0; i < 2*n + 1; i++) {
                dx = sig_pred.col(i) - Xk_pred;
                dz =  Zk_sig_pred.col(i) - Zk_pred;
                Tk += Wc(i) * (dx * dz.transpose());
            }

            Kk = Tk * Sk.inverse();
            Zk_actual << gpsx, gpsy, gpsw;
            while(Zk_actual(2) - Xk_pred(2) > M_PI) Zk_actual(2) -= 2 * M_PI;
            while(Zk_actual(2) - Xk_pred(2) < -M_PI) Zk_actual(2) += 2 * M_PI;
            
            Xk_cur = Xk_pred + Kk  * (Zk_actual - Zk_pred);
            Pk_cur = Pk_pred - Kk * Sk * Kk.transpose();

            while (Xk_cur(2) > M_PI) Xk_cur(2) -= 2 * M_PI;
            while (Xk_cur(2) < -M_PI) Xk_cur(2) += 2 * M_PI;

            robotx = Xk_cur(0);
            roboty = Xk_cur(1);
            robotw = Xk_cur(2) * TO_DEG;
            Xk_prev = Xk_cur;
            Pk_prev = Pk_cur;

            while (robotw > 180) robotw -= 360;
            while (robotw < -180) robotw += 360;

            odox_offset = odox_buf - robotx;
            odoy_offset = odoy_buf - roboty;
            odow_offset = odow_buf - robotw;
            offset_msg.posisi_x_offset = odox_offset;
            offset_msg.posisi_y_offset = odoy_offset;
            offset_msg.sudut_w_offset  = odow_offset;
            pub_robot_offset->publish(offset_msg);
        }

        // Publish Visuals
        geometry_msgs::msg::Pose pos_msg;
        pos_msg.position.x = robotx; pos_msg.position.y = roboty;
        tf2::Quaternion q; q.setRPY(0, 0, robotw * TO_RAD);
        pos_msg.orientation = tf2::toMsg(q);
        pub_robot_pos->publish(pos_msg);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map"; t.child_frame_id = "odom";
        t.transform.translation.x = robotx; t.transform.translation.y = roboty;
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster->sendTransform(t);

        // Sigma Point Markers
        visualization_msgs::msg::MarkerArray marker_array;
        for (int i = 0; i < 2 * n + 1; i++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "ukf_sigma";
            marker.id = marker_counter++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = sig_pred(0, i);
            marker.pose.position.y = sig_pred(1, i);
            marker.pose.position.z = 0.4;
            marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;
            marker.color.r = 1.0f; marker.color.a = 1.0f;
            marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fusion_ukf>();
    RCLCPP_INFO(node->get_logger(), "START FUSION UKF NODE !!!!");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}