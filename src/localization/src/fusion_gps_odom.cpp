#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

// Perbaikan: Gunakan huruf kecil semua untuk file header
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot_msgs/msg/kirim_offset_udp.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"

// Gunakan nama paket 'localization' yang baru
#include <localization/marvelmind_ros2.hpp>
#include "localization/fusion_gps_odom.h"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace Eigen;

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

#define r_robot_gps 0.0 
#define offset_gpsx 0.0 
#define offset_gpsy 0.0  
#define gps_f_sample 0.0

float odox_buf, odoy_buf, odow_buf;
float odox_offset, odoy_offset, odow_offset;
float gpsx_buf, gpsy_buf, gpsw_buf;
float gpsx, gpsy, gpsw;
float gpsx_prev, gpsy_prev, gpsw_prev;
float gps_quality, gps_q;

int status_init_pos;
float robotx, roboty, robotw;

float vx_lokal, vy_lokal, vw_lokal;
float vx_global, vy_global, vw_global;
float vx_gps, vy_gps, vw_gps;

float w_mean, w_mean_sin, w_mean_cos;
float beacon_address;

float yaw, yaw_degrees, theta_cmps;

class fusion : public rclcpp::Node 
{
    // Perbaikan: Gunakan Huruf Kapital di awal tipe data (KirimKecepatanUdp)
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
    fusion() : Node("fusion")
    {
        sub_robot_pos = this->create_subscription<udp_bot_msgs::msg::TerimaUdp>(
            "data_terima_udp", 10, std::bind(&fusion::robot_odom_callback, this, std::placeholders::_1));

        gps_pos_sub = this->create_subscription<marvelmind_msgs::msg::HedgePosAng>(
            "hedgehog_pos_ang", 20, std::bind(&fusion::robot_hedge_callback, this, std::placeholders::_1));

        gps_quality_sub = this->create_subscription<marvelmind_msgs::msg::HedgeQuality>(
            "hedgehog_quality", 20, std::bind(&fusion::hedge_quality_callback, this, std::placeholders::_1));

        imu_fusion_sub = this->create_subscription<marvelmind_msgs::msg::HedgeImuFusion>(
            "hedgehog_imu_fusion", 20, std::bind(&fusion::hedge_imu_fus_callback, this, std::placeholders::_1));
 
        pub_robot_offset = this->create_publisher<udp_bot_msgs::msg::KirimOffsetUdp>("offset_kirim_udp", 10);
        pub_robot_vel = this->create_publisher<udp_bot_msgs::msg::KirimKecepatanUdp>("kecepatan_kirim_udp", 10);
        pub_robot_pos = this->create_publisher<geometry_msgs::msg::Pose>("robot_position", 10);
        pub_robot_gps = this->create_publisher<geometry_msgs::msg::Pose>("robot_gps_position", 10);
        pub_robot_odo = this->create_publisher<geometry_msgs::msg::Pose>("robot_odometry", 10);

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        simple_nav_timer = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&fusion::fusion_process_callback, this)); 
    }

void robot_odom_callback(const udp_bot_msgs::msg::TerimaUdp::SharedPtr msg)
    {
        odox_buf = msg->posisi_x_buffer;
        odoy_buf = msg->posisi_y_buffer;
        odow_buf = msg->sudut_w_buffer;
        vx_lokal = msg->kecepatan_robotx;
        vw_lokal = msg->kecepatan_robotw;
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
    }

    void hedge_quality_callback(const marvelmind_msgs::msg::HedgeQuality::SharedPtr msg)
    {
        gps_quality = (float)(msg->quality_percents)/100;
        RCLCPP_INFO_STREAM(get_logger(), "GPS QUALITY: " << gps_quality);
    }

    void fusion_process_callback()
    {
        // Variabel ini diinisialisasi dari fusion_gps_odom.h
        gpsx = gpsx_buf;
        gpsy = gpsy_buf;
        gpsw = gpsw_buf;
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

                status_init_pos = 1;
                Xk_prev(0) = robotx;
                Xk_prev(1) = roboty;
                Xk_prev(2) = robotw;
            }
        }
        else 
        {
            Uk(0) = vx_global; Uk(1) = vy_global; Uk(2) = vw_global*TO_DEG;
            Xkp = A*Xk_prev + B*Uk + Wk;
            while (Xkp(2) > 180) Xkp(2) -= 360;
            while (Xkp(2) < -180) Xkp(2) += 360;

            Pkp = A*Pk_prev*A.transpose() + Qk;
            Pkp = Pkp.array() * I.array(); 

            Gps_Q(0,0) = exp(-(pow((gpsx-Xkp(0))/0.02,2)));
            Gps_Q(1,1) = exp(-(pow((gpsy-Xkp(1))/0.02,2)));
            Gps_Q(2,2) = exp(-(pow((gpsw-Xkp(2))/10.0,2)));
            
            R = (I-Gps_Q*H)*R_init + Gps_Q*(H*Pkp*H.transpose());
            Kg = H*Pkp*H.transpose() + R;
            Kg = Pkp*H.transpose() * Kg.inverse();

            yk(0) = gpsx; yk(1) = gpsy; yk(2) = theta_cmps* TO_DEG;
            Yk = C*yk + Zk;

            while(Yk(2) - Xkp(2) > 180){Yk(2) -= 360;}
            while(Yk(2) - Xkp(2) < -180){Yk(2) += 360;}

            Xk = Xkp + Kg*(Yk-H*Xkp);
            while (Xk(2) > 180) Xk(2) -= 360;
            while (Xk(2) < -180) Xk(2) += 360;

            Pk = (I - Kg*H) * Pkp;
            Xk_prev = Xk;
            Pk_prev = Pk;

            robotx = Xk(0);
            roboty = Xk(1);
            robotw = Xk(2);
            
            RCLCPP_INFO_STREAM(get_logger(), "X: " << robotx << " Y: " << roboty << " W: " << robotw);

            odox_offset = odox_buf - robotx;
            odoy_offset = odoy_buf - roboty;
            odow_offset = odow_buf - robotw;
            
            offset_msg.posisi_x_offset = odox_offset;
            offset_msg.posisi_y_offset = odoy_offset;
            offset_msg.sudut_w_offset  = odow_offset;
            pub_robot_offset->publish(offset_msg);
        }

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
        t.transform.translation.z = 0.0;
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster->sendTransform(t);

        geometry_msgs::msg::Pose gps_msg;
        gps_msg.position.x = gpsx;
        gps_msg.position.y = gpsy;
        tf2::Quaternion q_gps;
        q_gps.setRPY(0, 0, gpsw * TO_RAD);
        gps_msg.orientation = tf2::toMsg(q_gps);
        pub_robot_gps->publish(gps_msg);

        geometry_msgs::msg::Pose odo_msg;
        odo_msg.position.x = Xkp(0);
        odo_msg.position.y = Xkp(1);
        tf2::Quaternion q_odo;
        q_odo.setRPY(0, 0, Xkp(2) * TO_RAD);
        odo_msg.orientation = tf2::toMsg(q_odo);
        pub_robot_odo->publish(odo_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Perbaikan: Jangan buat dua node yang sama
    auto node = std::make_shared<fusion>();
    RCLCPP_INFO(node->get_logger(), "START FUSION NODE !!!!");

    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}