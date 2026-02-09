#include "client.hpp"


using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options) 
    : Node("client_node", options)
{   

    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

ClientNode::~ClientNode() {};
