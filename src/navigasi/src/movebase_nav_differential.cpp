#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MoveBaseNav : public rclcpp::Node
{
public:
    MoveBaseNav() : Node("movebase_nav_differential"),
                    tf_buffer_(get_clock()),
                    tf_listener_(tf_buffer_)
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal",
            10,
            std::bind(&MoveBaseNav::goalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&MoveBaseNav::odomCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MoveBaseNav::loop, this));

        RCLCPP_INFO(get_logger(), "Navigation node started");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Odometry odom_;

    bool goal_received_ = false;

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO(get_logger(), "Goal received");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
    }

    void loop()
    {
        if (!goal_received_)
            return;

        double dx = goal_.pose.position.x - odom_.pose.pose.position.x;
        double dy = goal_.pose.position.y - odom_.pose.pose.position.y;

        double distance = sqrt(dx * dx + dy * dy);

        geometry_msgs::msg::Twist cmd;

        if (distance > 0.1)
        {
            cmd.linear.x = 0.3;
            cmd.angular.z = atan2(dy, dx);
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            goal_received_ = false;
            RCLCPP_INFO(get_logger(), "Goal reached");
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveBaseNav>());
    rclcpp::shutdown();
    return 0;
}

