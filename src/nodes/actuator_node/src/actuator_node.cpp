#include "actuator_node.hpp"

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions& options) 
    : Node("actuator_node", options)
{   
    manager_ = std::make_unique<ActuatorManager>();
    manager_->init_node(this);

    auto& parameters = manager_->get_parameters();

    if (manager_->init_comm() != ActuatorError::OK)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial na porta %s.", parameters.usb_port.c_str());
        throw std::runtime_error("");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    actuator_subscriber_ = this->create_subscription<ActuatorGoalPosition>(
        parameters.base_name + "/goal_position", qos, 
        [this](const ActuatorGoalPosition::SharedPtr msg) {
            this->goal_position_callback(msg);
        });
    
    motor_service_ = this->create_service<SetMotorConfig>(
        parameters.base_name + "/command", 
        [this](const std::shared_ptr<SetMotorConfig::Request> req, 
                std::shared_ptr<SetMotorConfig::Response> res) {
            this->motor_service_callback(req, res);
        });

    // criando publishers
    for (auto id: parameters.actuator_ids)
    {
        auto topic =
            parameters.base_name + "/current_position/id_" +
            std::to_string(static_cast<int>(id));

        publishers_[id] =
            this->create_publisher<ActuatorCurrentPosition>(topic, qos);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters.update_rate_ms), 
        [this]() {
            publish_position_callback();
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    response->success = false;

    ActuatorError result = manager_->execute_command(
        static_cast<uint8_t>(request->id), 
        request->command, request->params);

    if (result == ActuatorError::OK) response->success = true;
}

void ActuatorNode::goal_position_callback(const ActuatorGoalPosition::SharedPtr msg)
{
    manager_->set_goal_position(
        static_cast<uint8_t>(msg->id),
        static_cast<uint16_t>(msg->goal));
}

void ActuatorNode::publish_position_callback()
{
    auto msg = ActuatorCurrentPosition();

    auto& parameters = manager_->get_parameters();
    for (auto id : parameters.actuator_ids)
    {
        uint16_t curr_pos;
        if (manager_->get_current_position(id, curr_pos) != ActuatorError::OK)
            continue;

        msg.id = id;
        msg.position = curr_pos;
        msg.stamp = this->get_clock()->now();

        publishers_[id]->publish(msg);
    }
}

ActuatorNode::~ActuatorNode() = default;