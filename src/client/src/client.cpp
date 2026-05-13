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
    
    this->declare_parameter("intervals", 100);
    this->declare_parameter("time_per_goal", 1000);
    this->declare_parameter("speed", 360);
    this->declare_parameter("angle_path", std::vector<int32_t>{-100, 900, -100, 900, -100});
    
    intervals_ = this->get_parameter("intervals").as_int();
    time_per_goal_ = this->get_parameter("time_per_goal").as_int();
    speed_ = this->get_parameter("speed").as_int();
    std::vector<long> angle_path_raw = this->get_parameter("angle_path").as_integer_array();
    
    angle_path_.clear();
    angle_path_.reserve(angle_path_raw.size());
    for (long angle : angle_path_raw)
        angle_path_.push_back(static_cast<int32_t>(angle));
    
    actuator_publisher_ = this->create_publisher<interfaces::msg::MG8008ECommand>("/actuator/command", 10);
    //actuator_subscriber_ = this->create_subscription<interfaces::msg::MG8008EState>("/actuator/state", 10);

    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
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
    int intervals = intervals_;
    int time_per_goal = time_per_goal_; // ms
    int time_step = time_per_goal / intervals; // tempo por intervalo
    std::vector<int32_t> angle_path = angle_path_;
    
    // mandando pra posiço inicial
    if (angle_path.size() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Enviando posicao %d.", (int)angle_path[0]);
        send_angle("joint_1", angle_path[0], speed_);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
            
    for (size_t i = 0; i < angle_path.size()-1; i++)
    {
        int32_t curr_angle = angle_path[i];
        int32_t next_angle = angle_path[i+1];
        int32_t step = (int32_t) ((next_angle-curr_angle)/intervals);
        
        RCLCPP_INFO(this->get_logger(), "\nAngulo alvo: %d\nPasso: %d\n", (int)next_angle, (int)step);

        for (size_t j = 0; j < intervals; j++)
        {
            curr_angle += step; 
            RCLCPP_INFO(this->get_logger(), "Enviando posicao %d.", (int)curr_angle);
            send_angle("joint_1", curr_angle, speed_);
            
            rclcpp::sleep_for(std::chrono::milliseconds(time_step)); 
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Trajetoria concluida");

}

ClientNode::~ClientNode() {}
