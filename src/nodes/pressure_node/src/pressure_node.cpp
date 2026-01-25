#include <vector>
#include <sstream>

#include "pressure_node.hpp"
#include "driver/pressure_factory.hpp"

using namespace std::chrono_literals;

PressureNode::PressureNode(const rclcpp::NodeOptions& options) 
    : Node("pressure_node", options)
{
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    pressure_drivers_.resize(parameter_manager_->get_ids().size());

    for (size_t idx = 0; idx < parameter_manager_->get_ids().size(); idx++)
    {
        pressure_drivers_[idx] = PressureFactory::create_pressure();

        if (pressure_drivers_[idx]->init(
            parameter_manager_->get_usb_ports()[idx], 
            parameter_manager_->get_baudrate()) < 0) {
                RCLCPP_FATAL(this->get_logger(), 
                    "Falha na inicialização do hardware serial na porta %s.", 
                    parameter_manager_->get_usb_ports()[idx].c_str());
                throw std::runtime_error("Falha ao inicializar PressureNode"); 
        }
    }
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    publisher_ = this->create_publisher<PressureState>(
        parameter_manager_->get_base_name() + "/state", qos);

    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        std::bind(&PressureNode::state_callback, this));
}

void PressureNode::state_callback()
{
    auto msg = PressureState();

    auto ids = parameter_manager_->get_ids();
    auto names = parameter_manager_->get_names();

    // evita segfault
    size_t min_size = std::min(ids.size(), names.size());

    msg.names.reserve(min_size);
    msg.pressures.reserve(min_size);

    uint8_t error_count = 0;
    for (size_t idx = 0; idx < min_size; idx++)
    {
        std::vector<uint16_t> data;
        
        if (pressure_drivers_[idx]->get_data(data) != 0)
        {
            error_count++;
            continue;
        }
        
        PressureData pd;
        pd.pressures.reserve(data.size());
        for (auto val : data)
            pd.pressures.push_back(static_cast<int16_t>(val));

        msg.pressures.push_back(pd);
        msg.names.push_back(parameter_manager_->get_names()[idx]);
        msg.stamp = this->get_clock()->now();
    }

    // se nenhuma palmilha enviou a posição
    // entao nao publica nada
    if (error_count < min_size) {
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publicado com %zu IDs", msg.names.size());
    }   
}

PressureNode::~PressureNode() = default;
