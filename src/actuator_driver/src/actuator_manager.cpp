#include "actuator_manager.hpp"

ActuatorManager::ActuatorManager() {}

void ActuatorManager::init_node(rclcpp::Node* node)
{
    declare_parameters(node);
    load_parameters(node);
}

void ActuatorManager::declare_parameters(rclcpp::Node* node)
{
    node->declare_parameter("base_name", "dxl");
    node->declare_parameter("usb_port", "/dev/ttyUSB0");
    node->declare_parameter("baudrate", 2000000);
    node->declare_parameter("actuator_ids", std::vector<int64_t>{1, 2, 3});
    node->declare_parameter("update_rate_ms", 15);
}

void ActuatorManager::load_parameters(rclcpp::Node* node)
{
    parameters_.base_name = node->get_parameter("base_name").as_string();
    parameters_.usb_port = node->get_parameter("usb_port").as_string();
    parameters_.baudrate = static_cast<uint32_t>(node->get_parameter("baudrate").as_int());
    parameters_.update_rate_ms = node->get_parameter("update_rate_ms").as_int();
    std::vector<long> raw_ids = node->get_parameter("actuator_ids").as_integer_array();
    
    // converte de long pra uint8_t
    parameters_.actuator_ids.clear();
    parameters_.actuator_ids.reserve(raw_ids.size()); // tamanho certo dos ids
    for (long id : raw_ids)
        if (id >= 0 && id <= 253) // valores suportados por uint8_t
            parameters_.actuator_ids.push_back(static_cast<uint8_t>(id)); 

}

int ActuatorManager::init_comm() {
    auto ctrl = ActuatorFactory::createDynamixel();
    controller_ = std::move(ctrl);
    
    if (controller_->init(parameters_.usb_port, parameters_.baudrate) < 0) {
        return -1; 
    }
    return 0;
}

int ActuatorManager::execute_command(
    rclcpp::Node* node, 
    uint8_t id, 
    const std::string& command, 
    const std::vector<int16_t>& params)
{
    if (!controller_) 
        return -1;

    if (!is_valid_id(id)) {
        RCLCPP_WARN(node->get_logger(), "ID inválido: %u", id);
        return -1;
    }

    if (command == "set_goal_position" && !params.empty()) {
        uint16_t goal = static_cast<uint16_t>(params[0]);
        return controller_->setGoalPosition(id, goal);
    } 
    else if (command == "set_torque" && !params.empty()) {
        uint16_t torque_status = static_cast<uint16_t>(params[0]);
        return controller_->setTorque(id, torque_status);
    }

    return -1;
}

int ActuatorManager::set_goal_position(
    rclcpp::Node* node, uint8_t id, uint16_t goal)
{
    if (!controller_) 
        return -1;

    if (!is_valid_id(id)) {
        RCLCPP_WARN(node->get_logger(), "ID inválido: %u", id);
        return -1;
    }

    if (goal > 4096) {
        RCLCPP_WARN(node->get_logger(), "Comando fora dos limites: ID %u Goal %u", id, goal);
        return -1;
    }
    
    if (controller_->setGoalPosition(id, goal) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Erro de comunicação ao mover atuador %u", id);
        return -1;
    } 

    RCLCPP_DEBUG(node->get_logger(), "Atuador %u movido para %u", id, goal);
    return 0;

}

int ActuatorManager::get_current_position(uint8_t id, uint16_t& curr_pos)
{
    if (!controller_) 
        return -1;

    if (!is_valid_id(id)) {
        return -1;
    }

    if (controller_->getCurrentPosition(id, curr_pos) < 0)
    {
        return -1;
    }

    return 0;
}

bool ActuatorManager::is_valid_id(uint8_t id) const
{
    return std::find(
        parameters_.actuator_ids.begin(),
        parameters_.actuator_ids.end(),
        id) != parameters_.actuator_ids.end();
}