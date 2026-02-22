#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

class UdpReceiverNode : public rclcpp::Node {
public:
    UdpReceiverNode() : Node("udp_receiver_node") {
        // Membuat publisher ke topik '/odom'
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Setup Socket UDP
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuat socket UDP!");
            return;
        }

        sockaddr_in servaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(8888); // Port UDP yang digunakan (bisa diubah)
        
        if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal melakukan bind ke Port 8888!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "UDP Receiver siap! Mendengarkan STM32 di Port 8888...");
        
        // Menjalankan thread terpisah agar tidak memblokir proses ROS 2
        receive_thread_ = std::thread(&UdpReceiverNode::receiveData, this);
    }
    
    ~UdpReceiverNode() {
        close(sockfd_);
        if(receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void receiveData() {
        char buffer[1024];
        while (rclcpp::ok()) {
            // Menunggu data masuk dari STM32
            int n = recvfrom(sockfd_, (char *)buffer, sizeof(buffer), MSG_WAITALL, nullptr, nullptr);
            if (n > 0) {
                buffer[n] = '\0'; // Menutup string
                
                // Asumsi format data dari STM32: "X,Y,YAW" (contoh: "1.5,0.2,3.14")
                float x = 0.0, y = 0.0, yaw = 0.0;
                if (sscanf(buffer, "%f,%f,%f", &x, &y, &yaw) == 3) {
                    publishOdometry(x, y, yaw);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Format data UDP tidak sesuai: %s", buffer);
                }
            }
        }
    }

    void publishOdometry(float x, float y, float yaw) {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        
        // Memasukkan data posisi X dan Y
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;
        
        // Mengubah sudut Yaw (Euler) menjadi Quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        
        // Publish data ke ROS 2
        odom_pub_->publish(msg);
    }

    int sockfd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::thread receive_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
