#include "client.hpp"

#include <vector>
#include <string>

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options) 
    : Node("client_node", options)
{   
    
    actuator_command_publisher_ = this->create_publisher<interfaces::msg::Command>("/dxl/command", 10);
    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
}

void ClientNode::execute_path() 
{
    std::vector<std::string> names = {"joint_1", "joint_2", "joint_3"};
    std::vector<std::vector<int16_t>> goals = 
    {
        {180, 180, 180}, 
        {190, 190, 190}, 
        {200, 200, 200}, 
        {210, 210, 210}, 
        {220, 220, 220}, 
        {230, 230, 230}
    };
    
    for (size_t idx = 0; idx < goals.size(); idx++) 
    {
        auto message = interfaces::msg::Command();
        message.names = names; // <n> nomes
        
        // converte de graus pra a unidade do dynamixel
        for (size_t i = 0; i < goals[idx].size(); i++)
        {
            message.goals.push_back(static_cast<int16_t>(std::round(goals[idx][i] / 0.088))); // <n> posicoes
        } 
        
        actuator_command_publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Mensagem publicada com %zu actuators", names.size());
        rclcpp::sleep_for(500ms);
    }
}

ClientNode::~ClientNode() {};
