#include "pressure_node.hpp"
#include "common_serial/serial_handler.hpp"

using namespace std::chrono_literals;

PressureNode::PressureNode() : Node("pressure_node")
{
    // carrega parametros
    load_parameters();
    
    auto qos = rclcpp::QoS(10).reliable();
    // cria um publisher pra cada sensor de pressao
    RCLCPP_INFO(this->get_logger(), 
        "Criando publisher para o sensor de pressao");
    auto publisher = this->create_publisher<InsoleData>("/pressure", qos);
    
    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_), 
        std::bind(&PressureNode::timer_callback, this));
}

// callback que envia os dados do sensor
void PressureNode::timer_callback()
{
    auto message = InsoleData();
    publisher_->publish(message);

}

void PressureNode::load_parameters()
{
    this->declare_parameter<int>("update_rate_ms", 15);
    set_parameters();
}

void PressureNode::set_parameters()
{
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
}

PressureNode::~PressureNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressureNode>());
    rclcpp::shutdown();
    return 0;
}
