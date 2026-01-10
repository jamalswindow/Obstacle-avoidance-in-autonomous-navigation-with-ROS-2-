#include "rclcpp/rclcpp.hpp"

class YourNode : public rclcpp::Node
{
public:
    YourNode() : Node("your_node")
    {
        RCLCPP_INFO(this->get_logger(), "Your Node has started!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YourNode>());
    rclcpp::shutdown();
    return 0;
}
