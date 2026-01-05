#include "pressure_manager.hpp"

PressureManager::PressureManager() {}

void PressureManager::init_node(rclcpp::Node* node)
{
    declare_parameters(node);
    load_parameters(node);
}

void PressureManager::declare_parameters(rclcpp::Node* node)
{
    node->declare_parameter("base_name", "dxl");
    node->declare_parameter("usb_ports", std::vector<std::string>{"/dev/ttyACM0", "/dev/ttyACM1"});
    node->declare_parameter("baudrate", 115200);
    node->declare_parameter("ids", std::vector<int64_t>{1, 2});
    node->declare_parameter("update_rate_ms", 15);
}

void PressureManager::load_parameters(rclcpp::Node* node)
{
    parameters_.base_name = node->get_parameter("base_name").as_string();
    parameters_.baudrate = static_cast<uint32_t>(node->get_parameter("baudrate").as_int());
    parameters_.update_rate_ms = node->get_parameter("update_rate_ms").as_int();
    std::vector<long> raw_ids = node->get_parameter("ids").as_integer_array();
    parameters_.usb_ports = node->get_parameter("usb_ports").as_string_array();

    parameters_.ids.clear();
    parameters_.ids.reserve(raw_ids.size());
    for (long id : raw_ids)
        if (id >= 0 && id <= 253) 
            parameters_.ids.push_back(static_cast<uint8_t>(id)); 
}

PressureError PressureManager::init_comm() {
    for (const auto& id : parameters_.ids)
    {
        controllers_[id] = PressureFactory::createPressure();

        if (controllers_[id]->init(parameters_.usb_ports[id], parameters_.baudrate) < 0) {
            return PressureError::NotInitialized; 
        }
    }

    return PressureError::OK;
}

PressureError PressureManager::get_data(uint8_t id, uint16_t& data)
{
    if (!is_valid_id(id))
        return PressureError::InvalidID;

    auto it = controllers_.find(id);
    if (it == controllers_.end())
        return PressureError::InvalidID;

    auto& controller = it->second;


    if (controller->getData(data) < 0)
        return PressureError::CommunicationError;

    return PressureError::OK;
}

bool PressureManager::is_valid_id(uint8_t id) const
{
    return std::find(
        parameters_.ids.begin(),
        parameters_.ids.end(),
        id) != parameters_.ids.end();
}