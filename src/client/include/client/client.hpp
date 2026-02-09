#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/command.hpp"

class ClientNode : public rclcpp::Node
{
    public:
        explicit ClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ClientNode();
                
        void run(int argc, char **argv);
        void execute_path(); 
        
    private:
        rclcpp::Publisher<interfaces::msg::Command>::SharedPtr actuator_command_publisher_;
};

#endif // CLIENT_HPP
