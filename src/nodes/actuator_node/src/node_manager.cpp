#include "node_manager.hpp"

NodeManager::NodeManager(
    std::shared_ptr<ActuatorManager> actuator_manager, 
    std::shared_ptr<ParameterManager> parameter_manager) 
    : actuator_manager_(actuator_manager), 
      parameter_manager_(parameter_manager) {}

ActuatorError NodeManager::init_serial() {
    auto ctrl = ActuatorFactory::createDynamixel();
    controller_ = std::move(ctrl);
    
    if (controller_->init(
        parameter_manager_->get_usb_port(), 
        parameter_manager_->get_baudrate()) < 0) {
            return ActuatorError::NotInitialized; 
    }
    return ActuatorError::OK;
}

ActuatorError NodeManager::set_torque(const std::vector<uint8_t>& ids, bool status) 
{
    if (!controller_) return ActuatorError::NotInitialized;

    for (uint8_t id : ids) 
    {
        if (actuator_manager_->get_actuator_by_id(id) == nullptr)
            return ActuatorError::InvalidID; 
    }

    for (uint8_t id : ids)
    {
        if (controller_->setTorque(id, status ? 1 : 0) != 0)
            return ActuatorError::CommunicationError;

        actuator_manager_->update_torque(id, status);
    }

    return ActuatorError::OK;
}

ActuatorError NodeManager::set_goal_position(
    const std::vector<uint8_t>& ids, 
    const std::vector<uint16_t>& positions)
{
    if (ids.size() != positions.size())
        return ActuatorError::InvalidParameter;

    if (!controller_) 
        return ActuatorError::NotInitialized;

    for (uint8_t id : ids) 
    {
        if (actuator_manager_->get_actuator_by_id(id) == nullptr)
            return ActuatorError::InvalidID; 
    }

    for (size_t idx = 0; idx < ids.size(); idx++)
    {
        if (controller_->setGoalPosition(ids[idx], positions[idx]) != 0)
            return ActuatorError::CommunicationError;

        actuator_manager_->update_position(ids[idx], positions[idx]);
    }

    return ActuatorError::OK;
}

ActuatorError NodeManager::get_current_position(    
    const std::vector<uint8_t>& ids, 
    std::vector<uint16_t>& positions)
{
    if (!controller_) 
        return ActuatorError::NotInitialized;

    for (uint8_t id : ids) 
    {
        if (actuator_manager_->get_actuator_by_id(id) == nullptr)
            return ActuatorError::InvalidID; 
    } 

    positions.resize(ids.size());
    for (size_t idx = 0; idx < ids.size(); idx++)
    {
        if (controller_->getCurrentPosition(ids[idx], positions[idx]) < 0)
            return ActuatorError::CommunicationError;

        // precisa setar a nova posição do atuador
        actuator_manager_->update_position(ids[idx], positions[idx]);
    }

    return ActuatorError::OK;
}