#include <vector>
#include <sstream>

#include "pressure_node.hpp"

#define MAX_BUFFER_COLLECT          6 * 16
#define BUFFER_SIZE                 8 * 16 * 5
#define DEVICE                      "/dev/ttyACM0"
#define DEFAULT_BAUDRATE                    115200

using namespace std::chrono_literals;

PressureNode::PressureNode() : Node("pressure_node")
{
    manager_ = std::make_unique<PressureManager>();
    manager_->init_node(this);

    auto& parameters = manager_->get_parameters();

    if (manager_->init_comm() != PressureError::OK)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial em uma da(s) porta(s)");
        throw std::runtime_error("");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    // CONSERTAR AQUI
    for (const auto &id : parameters.ids)
    {
        std::string topic = parameters.base_name + "/id_" + std::to_string(id);
        publishers_[id] = this->create_publisher<PressureData>(topic, qos);
    }
    
    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters.update_rate_ms), 
        std::bind(&PressureNode::timer_callback, this));
}

void PressureNode::timer_callback()
{
    auto parameters = manager_->get_parameters();
    for (const auto& id : parameters.ids)
    {
        uint16_t data;
        if (manager_->get_data(id, data) != PressureError::OK)
        {
            RCLCPP_WARN(this->get_logger(), "Erro ao buscar dados");
        }

        auto message = PressureData();
        message.stamp = this->get_clock()->now();
        message.pressures = { data };
        
        publishers_[id]->publish(message);
    }
}

PressureNode::~PressureNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
        
    rclcpp::spin(std::make_shared<PressureNode>());
    rclcpp::shutdown();
    return 0;
}