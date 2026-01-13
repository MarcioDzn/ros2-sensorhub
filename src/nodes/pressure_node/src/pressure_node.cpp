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
    publisher_ = this->create_publisher<PressureState>(parameters.base_name + "/state", qos);

    
    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters.update_rate_ms), 
        std::bind(&PressureNode::state_callback, this));
}

void PressureNode::state_callback()
{
    auto msg = PressureState();
    auto parameters = manager_->get_parameters();

    msg.ids.reserve(parameters.ids.size());
    msg.pressures.reserve(parameters.ids.size());

    uint8_t error_count = 0;
    for (const auto& id : parameters.ids)
    {
        std::vector<uint16_t> data;
        
        if (manager_->get_data(id, data) != PressureError::OK)
        {
            error_count++;
            continue;
        }
        
        PressureData pd;
        pd.pressures.reserve(data.size());
        for (auto val : data)
            pd.pressures.push_back(static_cast<int16_t>(val));

        msg.pressures.push_back(pd);
        msg.ids.push_back(static_cast<int8_t>(id));
        msg.stamp = this->get_clock()->now();
    }

    // se nenhuma palmilha enviou a posição
    // entao nao publica nada
    if (error_count < parameters.ids.size()) {
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "PUBLICANDO: %d", msg.ids[1]);
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