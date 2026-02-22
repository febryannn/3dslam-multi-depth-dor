#ifndef MARVELMIND_ROS2_HPP_
#define MARVELMIND_ROS2_HPP_

/*
Copyright (c) 2022, Marvelmind Robotics
All rights reserved.
*/

// C++ STL
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef WIN32
#include <semaphore.h>
#include <fcntl.h>
#endif

using namespace std::chrono_literals;

extern "C" 
{
    #include "marvelmind_hedge.h"
}

// ROS2
#include "rclcpp/rclcpp.hpp"

// Perbaikan Include: Menggunakan marvelmind_msgs dan snake_case
// Perbaiki bagian include ini sesuai hasil ls kamu
#include "marvelmind_msgs/msg/hedge_pos.hpp"         // Sebelumnya hedge_position.hpp
#include "marvelmind_msgs/msg/hedge_pos_a.hpp"       // Sebelumnya hedge_position_addressed.hpp
#include "marvelmind_msgs/msg/hedge_pos_ang.hpp"     // Sebelumnya hedge_position_angle.hpp
#include "marvelmind_msgs/msg/beacon_pos_a.hpp"      // Sebelumnya beacon_position_addressed.hpp

#include "marvelmind_msgs/msg/hedge_imu_raw.hpp"
#include "marvelmind_msgs/msg/hedge_imu_fusion.hpp"
#include "marvelmind_msgs/msg/hedge_telemetry.hpp"
#include "marvelmind_msgs/msg/hedge_quality.hpp"
#include "marvelmind_msgs/msg/beacon_distance.hpp"
#include "marvelmind_msgs/msg/marvelmind_waypoint.hpp"
#include "marvelmind_msgs/msg/marvelmind_user_data.hpp"
class marvelmind_ros2 : public rclcpp::Node
{
    public:
        marvelmind_ros2();
        ~marvelmind_ros2();

    private:
        // Publishers menggunakan tipe data dari marvelmind_msgs
        rclcpp::Publisher<marvelmind_msgs::msg::HedgePosAng>::SharedPtr hedge_pos_ang_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::HedgePosA>::SharedPtr hedge_pos_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::HedgePos>::SharedPtr hedge_pos_noaddress_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::BeaconPosA>::SharedPtr beacons_pos_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::HedgeImuRaw>::SharedPtr hedge_imu_raw_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::HedgeImuFusion>::SharedPtr hedge_imu_fusion_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::BeaconDistance>::SharedPtr beacon_distance_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::HedgeTelemetry>::SharedPtr hedge_telemetry_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::HedgeQuality>::SharedPtr hedge_quality_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::MarvelmindWaypoint>::SharedPtr marvelmind_waypoint_publisher;
        rclcpp::Publisher<marvelmind_msgs::msg::MarvelmindUserData>::SharedPtr marvelmind_user_data_publisher;

        rclcpp::TimerBase::SharedPtr marvelmind_ros2_pub_timer;

        // Function prototypes
        int hedgeReceivePrepare();
        bool hedgeReceiveCheck(void);
        bool beaconReceiveCheck(void);
        bool hedgeIMURawReceiveCheck(void);
        bool hedgeIMUFusionReceiveCheck(void);
        void getRawDistance(uint8_t index);
        bool hedgeTelemetryUpdateCheck(void);
        bool hedgeQualityUpdateCheck(void);  
        bool marvelmindWaypointUpdateCheck(void);
        bool marvelmindUserDataUpdateCheck(void);
        void publishTimerCallback();

        // Variables
        std::string tty_filename;
        int publish_rate_in_hz;
        
        #ifdef WIN32
        int tty_baudrate;
        #else
        unsigned int tty_baudrate;
        #endif

        struct MarvelmindHedge * hedge = NULL;
        int64_t hedge_timestamp_prev = 0;

        // Setup empty messages untuk pengiriman
        marvelmind_msgs::msg::HedgePos hedge_pos_noaddress_msg;
        marvelmind_msgs::msg::HedgePosA hedge_pos_msg;
        marvelmind_msgs::msg::HedgePosAng hedge_pos_ang_msg;
        marvelmind_msgs::msg::BeaconPosA beacon_pos_msg;
        marvelmind_msgs::msg::HedgeImuRaw hedge_imu_raw_msg;
        marvelmind_msgs::msg::HedgeImuFusion hedge_imu_fusion_msg;
        marvelmind_msgs::msg::BeaconDistance beacon_raw_distance_msg;
        marvelmind_msgs::msg::HedgeTelemetry hedge_telemetry_msg;
        marvelmind_msgs::msg::HedgeQuality hedge_quality_msg;
        marvelmind_msgs::msg::MarvelmindWaypoint marvelmind_waypoint_msg;
        marvelmind_msgs::msg::MarvelmindUserData marvelmind_user_data_msg;
};

#endif // MARVELMIND_ROS2_HPP_