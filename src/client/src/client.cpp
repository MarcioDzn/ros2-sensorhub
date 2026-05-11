#include "client.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options)
    : Node("client_node", options)
{
    actuator_publisher_ = this->create_publisher<interfaces::msg::MG8008ECommand>("/actuator/command", 10);

    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
}


// Função principal de execução
void ClientNode::execute_path()
{
    std::vector<int32_t> angle_path = {-100, 900, -100, 900, -100};
    for (size_t i = 0; i < angle_path.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Enviando posicao.");
        interfaces::msg::MG8008ECommand actuator_msg;
        actuator_msg.names.push_back("joint_1");
        actuator_msg.angles.push_back(angle_path[i]);
        actuator_msg.speeds.push_back(360);
        actuator_publisher_->publish(actuator_msg);
        RCLCPP_INFO(this->get_logger(), "Posicao enviada.");
        
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
    
    RCLCPP_INFO(this->get_logger(), "Trajetoria concluida");

}

ClientNode::~ClientNode() {}
