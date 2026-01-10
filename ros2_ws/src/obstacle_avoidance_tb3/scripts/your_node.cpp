#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleAvoidance::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        RCLCPP_INFO(this->get_logger(), "Closest Obstacle: %.2f meters", min_distance);

        geometry_msgs::msg::Twist cmd;
        if (min_distance < 0.5) {
            cmd.linear.x = 0.0;  // Stop
            cmd.angular.z = 0.5;  // Turn left
        } else {
            cmd.linear.x = 0.2;  // Move forward
            cmd.angular.z = 0.0;  // Go straight
        }
        pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}

