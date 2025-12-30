#include "actuator_node.hpp"

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode() 
    : Node("actuator_node")
{   
    manager_ = std::make_unique<ActuatorManager>();
    manager_->init_node(this);

    if (manager_->init_comm() < 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Falha na inicialização do hardware serial.");
        throw std::runtime_error("Falha na inicialização do hardware serial.");
    }

    auto parameters = manager_->get_parameters();
    actuator_subscriber_ = this->create_subscription<ActuatorGoalPosition>(
        parameters.base_name + "/goal_position", 10, 
        [this](const ActuatorGoalPosition::SharedPtr msg) {
            this->goal_position_callback(msg);
        });
    
    motor_service_ = this->create_service<SetMotorConfig>(
        parameters.base_name + "/motor_config", 
        [this](const std::shared_ptr<SetMotorConfig::Request> req, 
                std::shared_ptr<SetMotorConfig::Response> res) {
            this->motor_service_callback(req, res);
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    response->success = false;

    int result = manager_->execute_command(this, 
        static_cast<uint8_t>(request->id), 
        request->command, request->params);

    if (result == 0) response->success = true;
}

void ActuatorNode::goal_position_callback(const ActuatorGoalPosition::SharedPtr msg)
{
    manager_->set_goal_position(this,
        static_cast<uint8_t>(msg->id),
        static_cast<uint16_t>(msg->goal));
}

ActuatorNode::~ActuatorNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);    
    rclcpp::spin(std::make_shared<ActuatorNode>());
    rclcpp::shutdown();
    return 0;
}