#include "rclcpp/rclcpp.hpp"
#include "client.hpp" 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    node->run(argc, argv);
    rclcpp::shutdown();
    return 0;
}
