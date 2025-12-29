#include "actuator_manager.hpp"

ActuatorManager::ActuatorManager(std::shared_ptr<rclcpp::Node>& node_ptr) {
    parameters_ = std::make_unique<ActuatorParams>();

    declare_parameters(node_ptr);
    set_parameters(node_ptr);
}

void ActuatorManager::declare_parameters(std::shared_ptr<rclcpp::Node>& node_ptr)
{
    node_ptr->declare_parameter("usb_port", "/dev/ttyUSB0");
    node_ptr->declare_parameter("baudrate", 2000000);
    node_ptr->declare_parameter("actuator_ids", {1, 2, 3});
    node_ptr->declare_parameter("update_rate_ms", 15);
}

void ActuatorManager::set_parameters(std::shared_ptr<rclcpp::Node>& node_ptr)
{
    parameters_->usb_port = node_ptr->get_parameter("usb_port").as_string();
    parameters_->baudrate = static_cast<uint32_t>(
        node_ptr->get_parameter("baudrate").as_int());
    parameters_->update_rate_ms = node_ptr->get_parameter("update_rate_ms").as_int();
    std::vector<long> raw_ids = node_ptr->get_parameter("actuator_ids").as_integer_array();
    
    // converte de long pra uint8_t
    parameters_->actuator_ids.clear();
    parameters_->actuator_ids.reserve(raw_ids.size()); // tamanho certo dos ids
    for (long id : raw_ids)
        if (id >= 0 && id <= 253) // valores suportados por uint8_t
            parameters_->actuator_ids.push_back(static_cast<uint8_t>(id)); 

}

int ActuatorManager::init_comm(std::shared_ptr<rclcpp::Node>& node_ptr) {
    auto ctrl = ActuatorFactory::createDynamixel();
    controller_ = std::shared_ptr<ActuatorController>(std::move(ctrl));
    
    if (controller_->init(parameters_->usb_port, parameters_->baudrate) < 0) {
        RCLCPP_ERROR(node_ptr->get_logger(), 
            "FALHA AO ABRIR PORTA SERIAL: %s", parameters_->usb_port.c_str());
        return -1; 
    }

    RCLCPP_INFO(node_ptr->get_logger(), "Comunicação serial aberta com sucesso: %s", parameters_->usb_port.c_str());
    return 0;
}

int ActuatorManager::execute_command(
    std::shared_ptr<rclcpp::Node>& node_ptr, uint8_t id, 
    const std::string& command, const std::vector<int16_t>& params)
{
    if (!controller_) 
        return -1;

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
    std::shared_ptr<rclcpp::Node>& node_ptr, uint8_t id, uint16_t goal)
{
    if (goal > 4096) {
        RCLCPP_WARN(node_ptr->get_logger(), "Comando fora dos limites: ID %u Goal %u", id, goal);
        return -1;
    }
    
    if (controller_->setGoalPosition(id, goal) != 0) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Erro de comunicação ao mover atuador %u", id);
        return -1;
    } 

    RCLCPP_DEBUG(node_ptr->get_logger(), "Atuador %u movido para %u", id, goal);
    return 0;

}