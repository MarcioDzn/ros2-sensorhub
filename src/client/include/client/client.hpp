#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/actuator_state.hpp"

#include "interfaces/msg/mg8008_e_command.hpp"
#include "interfaces/msg/mg8008_e_state.hpp"

class ClientNode : public rclcpp::Node
{
    public:
        explicit ClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ClientNode();
                
        void run(int argc, char **argv);
        void execute_path(); 
        
    private:
        rclcpp::Publisher<interfaces::msg::MG8008ECommand>::SharedPtr actuator_publisher_;
        rclcpp::Subscription<interfaces::msg::MG8008EState>::SharedPtr actuator_subscriber_;
};

#endif // CLIENT_HPP
