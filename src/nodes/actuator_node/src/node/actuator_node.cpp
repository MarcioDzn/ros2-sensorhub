#include "node/actuator_node.hpp"

#include "control/node/parameter_manager.hpp"
#include "control/model/actuator_manager.hpp"

#include "driver/actuator_factory.hpp"

#include <vector>

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions& options) 
    : Node("actuator_node", options)
{   
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    actuator_driver_ = ActuatorFactory::createDynamixel();

    if (actuator_driver_->init(
        parameter_manager_->get_usb_port(), 
        parameter_manager_->get_baudrate()) != 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial na porta %s.", 
            parameter_manager_->get_usb_port().c_str());
        throw std::runtime_error("Falha ao inicializar ActuatorNode");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    actuator_subscriber_ = this->create_subscription<Command>(
        parameter_manager_->get_base_name() + "/command", qos, 
        [this](const Command::SharedPtr msg) {
            
            this->goal_position_callback(msg);
        });
    
    set_torque_service_ = this->create_service<SetTorque>(
        parameter_manager_->get_base_name() + "/set_torque", 
        [this](const std::shared_ptr<SetTorque::Request> req, 
                std::shared_ptr<SetTorque::Response> res) {
            this->set_torque_service_callback(req, res);
        });

    state_publisher_ = this->create_publisher<State>(
        parameter_manager_->get_base_name() + "/state", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        [this]() {
            state_callback();
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::set_torque_service_callback(
    const std::shared_ptr<SetTorque::Request> request,
    std::shared_ptr<SetTorque::Response> response)
{
    std::lock_guard<std::mutex> lock(driver_mutex_);

    response->success = false;
    // TODO: verificar se id existe
    auto result = actuator_driver_->set_torque(request->id, request->status ? 1 : 0);

    if (result != 0) {
        RCLCPP_WARN(this->get_logger(), 
            "Falha na configuração do torque. Erro: %d", 
            static_cast<int>(result));
        response->success = false;
        return; 
    }
    
    response->success = true;
}

void ActuatorNode::goal_position_callback(const Command::SharedPtr msg)
{
    if (msg->ids.empty() || msg->goals.empty()) return;

    // TODO: verificar se id existe
    size_t n = std::min(msg->ids.size(), msg->goals.size()); // menor tamanho

    std::lock_guard<std::mutex> lock(driver_mutex_);
    
    for (size_t idx = 0; idx < n; idx++)
    {
        auto result = actuator_driver_->set_goal_position(msg->ids[idx], msg->goals[idx]);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha no envio de Goal Position para o atuador %d. Erro: %d", 
            static_cast<int>(msg->ids[idx]), static_cast<int>(result));
        }
    }
}

void ActuatorNode::state_callback()
{
    State msg;

    std::vector<uint8_t> ids = parameter_manager_->get_ids();
    std::vector<std::string> names = parameter_manager_->get_names();
    std::vector<int16_t> positions(ids.size());

    {
        std::lock_guard<std::mutex> lock(driver_mutex_);

        // TODO: verificar se id existe
        for (size_t idx = 0; idx < ids.size(); idx++)
        {
            uint16_t temp_pos;
            auto result = actuator_driver_->get_current_position(ids[idx], temp_pos);
            positions[idx] = static_cast<int16_t>(temp_pos);

            if (result != 0) {
                RCLCPP_ERROR(this->get_logger(), "Falha na leitura do atuador %s. Erro: %d", 
                names[idx].c_str(), static_cast<int>(result));
            }
        }
    }

    msg.names.assign(names.begin(), names.end());
    msg.positions.assign(positions.begin(), positions.end());

    msg.stamp = this->get_clock()->now();
    state_publisher_->publish(msg); 
}

ActuatorNode::~ActuatorNode() = default;