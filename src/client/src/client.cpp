#include "client.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options)
    : Node("client_node", options)
{
    
    this->declare_parameter("amplitude", 500);
    this->declare_parameter("period", 5);
    this->declare_parameter("offset", 400);
    this->declare_parameter("phase", 0);
    this->declare_parameter("samples", 100);
    this->declare_parameter("speed", 360);
    
    amplitude_ = this->get_parameter("amplitude").as_int();
    period_ = this->get_parameter("period").as_int();
    offset_ = this->get_parameter("offset").as_int();
    phase_ = this->get_parameter("phase").as_double();
    samples_ = this->get_parameter("samples").as_int();
    speed_ = this->get_parameter("speed").as_int();
    
    actuator_publisher_ = this->create_publisher<interfaces::msg::MG8008ECommand>("/actuator/command", 10);
    actuator_subscriber_ = this->create_subscription<interfaces::msg::MG8008EState>(
        "/actuator/state", 
        1, 
        std::bind(&ClientNode::get_angle, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
}

double ClientNode::seno(int amplitude, int period, int offset, double phase, double t)
{
    double freq = 1.0 / period;
    return amplitude * std::sin(2 * 3.1415 * freq * t + phase) + offset;
}

void ClientNode::get_angle(const interfaces::msg::MG8008EState::SharedPtr msg)
{
        RCLCPP_INFO(this->get_logger(), "Angulo atuador %s: %d", msg->names[0].c_str(), msg->angles[0]);
}

void ClientNode::send_angle(std::string name, int32_t angle, int32_t speed) {
    interfaces::msg::MG8008ECommand actuator_msg;
    actuator_msg.names.push_back(name);
    actuator_msg.angles.push_back(angle);
    actuator_msg.speeds.push_back(speed);
    actuator_publisher_->publish(actuator_msg);
}

// Função principal de execução
void ClientNode::execute_path()
{   
    double t = 0;
    double interval = (double)period_ / samples_; // m

    // mandando pra posiço inicial
    RCLCPP_INFO(this->get_logger(), "Enviando posicao %d.", seno(amplitude_, period_, offset_, phase_, t));
    send_angle(
        "joint_1", 
        seno(amplitude_, period_, offset_, phase_, t), 
        speed_
    );

    rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
            
    for (size_t i = 1; i < samples_; i++)
    {
        t = i * interval;
        send_angle(
            "joint_1", 
            seno(amplitude_, period_, offset_, phase_, t), 
            speed_
        );

        rclcpp::sleep_for(std::chrono::milliseconds((int(interval*1000)))); 
    }
    rclcpp::spin_some(this->get_node_base_interface());
    RCLCPP_INFO(this->get_logger(), "Trajetoria concluida");
}

ClientNode::~ClientNode() {}
