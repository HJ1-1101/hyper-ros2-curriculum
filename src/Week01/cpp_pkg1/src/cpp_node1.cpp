#include "rclcpp/rclcpp.hpp"

class CppNode1 : public rclcpp::Node
{
    public:
        CppNode1() : Node("cpp_node1")
        {
            RCLCPP_INFO(this->get_logger(), "Hello from cpp_node1!");
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppNode1>());
    rclcpp::shutdown();
    return 0;
}