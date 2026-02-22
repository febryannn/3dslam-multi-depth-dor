#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

// Include pesan custom kamu (untuk robot asli)
#include "udp_bot_msgs/msg/kirim_kecepatan_udp.hpp"
// Include pesan standar ROS (untuk Gazebo)
#include "geometry_msgs/msg/twist.hpp" 

const std::string msg_teleop = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d

no keypress = stop

w/s : forward/backward
a/d : rotate left/rotate right
p/o : increase/decrease speeds

CTRL-C to quit
)";

float velx = 0.0;
float vely = 0.0;
float velw = 0.0;
float speed = 0.15;
float omega = 0.5;
float servo_ang = 0.0;

class TeleopRobot : public rclcpp::Node {
    // Publisher untuk Robot Asli
    rclcpp::Publisher<udp_bot_msgs::msg::KirimKecepatanUdp>::SharedPtr pub_robot_vel;
    // Publisher untuk Gazebo
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gazebo_vel; 

    udp_bot_msgs::msg::KirimKecepatanUdp msg_udp;
    geometry_msgs::msg::Twist msg_twist;

public:
    TeleopRobot() : Node("teleop_robot") {
        // 1. Publisher Custom (Kecepatan UDP)
        pub_robot_vel = this->create_publisher<udp_bot_msgs::msg::KirimKecepatanUdp>(
            "kecepatan_kirim_udp", 10);
            
        // 2. Publisher Standar (CMD_VEL untuk Gazebo)
        pub_gazebo_vel = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
    }

    void update_velocity(char key) {
        // Logika pembacaan tombol (Sama seperti punya kamu)
        if (key == 'w' || key == 'W') {
            velx = speed; vely = 0.0; velw = 0.0;
        } else if (key == 's' || key == 'S') {
            velx = -speed; vely = 0.0; velw = 0.0;
        } else if (key == 'a' || key == 'A') {
            velx = 0.0; vely = 0.0; velw = omega; // Kiri positif
        } else if (key == 'd' || key == 'D') {
            velx = 0.0; vely = 0.0; velw = -omega; // Kanan negatif
        } else if (key == 'p' || key == 'P') {
            speed += 0.05; if (speed > 0.3) speed = 0.3;
            omega += 0.1; if (omega > 0.8) omega = 0.8;
            std::cout << "Speed: " << speed << " | Turn: " << omega << std::endl;
            return; // Jangan publish kalau cuma ganti speed
        } else if (key == 'o' || key == 'O') {
            speed -= 0.05; if (speed < 0.15) speed = 0.15;
            omega -= 0.1; if (omega < 0.5) omega = 0.5;
            std::cout << "Speed: " << speed << " | Turn: " << omega << std::endl;
            return;
        } else {
            // Stop jika tombol lain ditekan (atau logika release key di main)
            // Note: Kode original kamu tidak auto-stop saat key dilepas,
            // tapi akan stop kalau menekan tombol sembarang selain WASD.
            velx = 0.0; vely = 0.0; velw = 0.0;
        }

        // --- KIRIM KE ROBOT ASLI (UDP) ---
        msg_udp.kecepatan_x      = velx;
        msg_udp.kecepatan_y      = vely;
        msg_udp.kecepatan_sudut  = velw;
        msg_udp.sudut_servo      = servo_ang;
        pub_robot_vel->publish(msg_udp);

        // --- KIRIM KE GAZEBO (/cmd_vel) ---
        msg_twist.linear.x  = velx;
        msg_twist.linear.y  = 0.0;
        msg_twist.linear.z  = 0.0;
        msg_twist.angular.x = 0.0;
        msg_twist.angular.y = 0.0;
        msg_twist.angular.z = velw; // Kecepatan putar masuk ke Z
        pub_gazebo_vel->publish(msg_twist);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopRobot>(); // Perbaikan: Pakai pointer agar update_velocity bisa dipanggil

    std::cout << msg_teleop << std::endl;

    // Setting terminal ke mode raw (non-canonical)
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Set non-blocking read agar loop rclcpp::spin tidak macet menunggu keyboard
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    while (rclcpp::ok()) {
        int ch = getchar();
        
        if (ch != EOF) {
            node->update_velocity(static_cast<char>(ch));
            // Tambahkan sedikit delay agar tidak spamming pesan terlalu cepat
            // usleep(50000); 
        } 
        
        // Agar node tetap responsif terhadap callback lain (jika ada)
        rclcpp::spin_some(node);
        usleep(10000); // Sleep 10ms untuk mengurangi beban CPU
    }

    // Kembalikan setting terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    rclcpp::shutdown();
    return 0;
}
