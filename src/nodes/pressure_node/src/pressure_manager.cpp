#include "pressure_manager.hpp"

PressureManager::PressureManager(
    std::shared_ptr<ParameterManager> parameter_manager)
    : parameter_manager_(parameter_manager) {}

PressureError PressureManager::init_comm() {
    for (const auto& id : parameter_manager_->get_ids())
    {
        controllers_[id] = PressureFactory::createPressure();

        if (controllers_[id]->init(
            parameter_manager_->get_usb_ports()[id], 
            parameter_manager_->get_baudrate()) < 0) {
                return PressureError::NotInitialized; 
        }
    }
    return PressureError::OK;
}

PressureError PressureManager::get_data(uint8_t id, std::vector<uint16_t>& data)
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
        parameter_manager_->get_ids().begin(),
        parameter_manager_->get_ids().end(),
        id) != parameter_manager_->get_ids().end();
}