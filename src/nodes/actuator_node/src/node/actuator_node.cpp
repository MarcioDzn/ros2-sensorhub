#include "node/actuator_node.hpp"

#include "control/node/parameter_manager.hpp"
#include "control/model/actuator_manager.hpp"
#include "node/node_manager.hpp"

#include <vector>

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions& options) 
    : Node("actuator_node", options)
{   
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    auto actuator_manager = std::make_shared<ActuatorManager>(parameter_manager_->get_ids());

    node_manager_ = std::make_unique<NodeManager>(
        actuator_manager, parameter_manager_);

    if (node_manager_->init_serial() != ActuatorError::OK)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial na porta %s.", 
            parameter_manager_->get_usb_port().c_str());
        throw std::runtime_error("");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
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
    response->success = false;
    auto result = node_manager_->set_torque({request->id}, request->status);
    if (result != ActuatorError::OK) {
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

    std::vector<uint8_t> ids(msg->ids.begin(), msg->ids.end());
    std::vector<uint16_t> goals(msg->goals.begin(), msg->goals.end());

    auto result = node_manager_->set_goal_position(ids, goals);
    if (result != ActuatorError::OK) {
        RCLCPP_WARN(this->get_logger(), "Falha no envio de Goal Positions. Erro: %d", static_cast<int>(result));
        return; 
    }
}

void ActuatorNode::state_callback()
{
    auto msg = State();
    std::vector<uint16_t> positions;
    std::vector<uint8_t> ids = parameter_manager_->get_ids();

    auto result = node_manager_->get_current_position(ids, positions);
    
    if (result != ActuatorError::OK) {
        RCLCPP_WARN(this->get_logger(), "Falha na leitura. Erro: %d", static_cast<int>(result));
        return; 
    }

    msg.ids.assign(ids.begin(), ids.end());
    msg.positions.assign(positions.begin(), positions.end());

    msg.stamp = this->get_clock()->now();
    state_publisher_->publish(msg); 
}

ActuatorNode::~ActuatorNode() = default;