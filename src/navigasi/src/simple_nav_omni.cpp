#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimpleNavOmni : public rclcpp::Node
{
public:
    SimpleNavOmni() : Node("simple_nav_omni")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10,
            std::bind(&SimpleNavOmni::goalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&SimpleNavOmni::odomCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimpleNavOmni::loop, this));

        RCLCPP_INFO(get_logger(), "Simple Omni Navigator started");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Odometry odom_;
    bool goal_received_ = false;

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = *msg;
        goal_received_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
    }

    void loop()
    {
        if (!goal_received_) return;

        double dx = goal_.pose.position.x - odom_.pose.pose.position.x;
        double dy = goal_.pose.position.y - odom_.pose.pose.position.y;
        double dist = hypot(dx, dy);

        geometry_msgs::msg::Twist cmd;

        if (dist > 0.1)
        {
            cmd.linear.x = dx;
            cmd.linear.y = dy;
        }
        else
        {
            goal_received_ = false;
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SimpleNavOmni>());
    rclcpp::shutdown();
}

