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

    actuator_subscriber_ = this->create_subscription<Command>(
        parameters.base_name + "/command", qos, 
        [this](const Command::SharedPtr msg) {
            this->goal_position_callback(msg);
        });
    
    set_torque_service_ = this->create_service<SetTorque>(
        parameters.base_name + "/set_torque", 
        [this](const std::shared_ptr<SetTorque::Request> req, 
                std::shared_ptr<SetTorque::Response> res) {
            this->set_torque_service_callback(req, res);
        });


    state_publisher_ = this->create_publisher<State>(
            parameters.base_name + "/state", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters.update_rate_ms), 
        [this]() {
            state_callback();
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::set_torque_service_callback(
    const std::shared_ptr<SetTorque::Request> request,
    std::shared_ptr<SetTorque::Response> response)
{
    ActuatorError result = manager_->set_torque(
        static_cast<uint8_t>(request->id), request->status);
    
    response->success = false;
    if (result == ActuatorError::OK) 
        response->success = true;
}

void ActuatorNode::goal_position_callback(const Command::SharedPtr msg)
{
    const std::vector<int8_t> & ids = msg->ids;
    const std::vector<int16_t> & goals = msg->goals;

    if (ids.empty())
        return;

    // cada goal deve estar associado a um atuador
    if (goals.size() != ids.size())
        return;

    // envio sequêncial para evitar concorrencia
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (manager_->set_goal_position(
                static_cast<uint8_t>(ids[i]), 
                static_cast<uint16_t>(goals[i])) != ActuatorError::OK)
            RCLCPP_ERROR(this->get_logger(), 
                "Erro ao mover motor %d para a posicao %d", 
                static_cast<int>(ids[i]), static_cast<int>(goals[i]));
    }
}

void ActuatorNode::state_callback()
{
    auto msg = State();

    auto& parameters = manager_->get_parameters();

    msg.ids.reserve(parameters.actuator_ids.size());
    msg.positions.reserve(parameters.actuator_ids.size());

    for (auto id : parameters.actuator_ids)
    {
        uint16_t curr_pos;
        if (manager_->get_current_position(id, curr_pos) != ActuatorError::OK)
            continue;

        msg.ids.push_back(static_cast<int8_t>(id));
        msg.positions.push_back(static_cast<int16_t>(curr_pos));
        msg.stamp = this->get_clock()->now();
    }

    state_publisher_->publish(msg);
}

ActuatorNode::~ActuatorNode() = default;