#include "client.hpp"


#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

// funcao auxiliar para split de strings
std::vector<std::string> split_string(const std::string &s, char delimiter)
{
    std::vector<std::string> result;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (!token.empty())
            result.push_back(token);
    }
    return result;
}

// funcao auxiliar para converter string para int16_t
std::vector<int16_t> split_ints(const std::string &s, char delimiter)
{
    std::vector<int16_t> result;
    auto tokens = split_string(s, delimiter);
    for (auto &t : tokens)
        result.push_back(static_cast<int16_t>(std::stoi(t)));
    return result;
}

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options) 
    : Node("client_node", options)
{   
    
    actuator_command_publisher_ = this->create_publisher<interfaces::msg::Command>("/dxl/command", 10);
    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    std::vector<std::string> names;
    std::vector<int16_t> goals;

    // Parse CLI simples
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--names" && i + 1 < argc)
            names = split_string(argv[++i], ',');
        else if (arg == "--goals" && i + 1 < argc)
            goals = split_ints(argv[++i], ',');
    }

    if (names.empty() || goals.empty() || names.size() != goals.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Use: --names nome1,nome2 --goals 10,20 (mesmo tamanho)");
        return;
    }

    auto message = interfaces::msg::Command();
    message.names = names;
    message.goals = goals;

    actuator_command_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Mensagem publicada com %zu actuators", names.size());
}

ClientNode::~ClientNode() {};
