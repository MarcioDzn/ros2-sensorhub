#include "node_manager.hpp"

NodeManager::NodeManager(
    std::shared_ptr<ActuatorManager> actuator_manager, 
    std::shared_ptr<ParameterManager> parameter_manager) 
    : actuator_manager_(actuator_manager), 
      parameter_manager_(parameter_manager) {}

ActuatorError NodeManager::init_serial() {
    auto ctrl = ActuatorFactory::createDynamixel();
    controller_ = std::move(ctrl);
    
    if (controller_->init(parameters_.usb_port, parameters_.baudrate) < 0) {
        return ActuatorError::NotInitialized; 
    }
    return ActuatorError::OK;
}

ActuatorError NodeManager::set_torque(uint8_t id, uint8_t status) 
{
    if (!controller_) 
        return ActuatorError::NotInitialized;

    if (!is_valid_id(id))
        return ActuatorError::InvalidID;

    if (controller_->setTorque(id, status) != 0)
        return ActuatorError::CommunicationError;

    return ActuatorError::OK;
}

ActuatorError NodeManager::set_goal_position(uint8_t id, uint16_t goal)
{
    if (!controller_) 
        return ActuatorError::NotInitialized;

    if (!is_valid_id(id))
        return ActuatorError::InvalidID;

    if (goal > 4096)
        return ActuatorError::InvalidParameter;
    
    if (controller_->setGoalPosition(id, goal) != 0)
        return ActuatorError::CommunicationError;

    return ActuatorError::OK;

}

ActuatorError NodeManager::get_current_position(uint8_t id, uint16_t& curr_pos)
{
    if (!controller_) 
        return ActuatorError::NotInitialized;

    if (!is_valid_id(id))
        return ActuatorError::InvalidID;

    if (controller_->getCurrentPosition(id, curr_pos) < 0)
        return ActuatorError::CommunicationError;

    return ActuatorError::OK;
}

bool NodeManager::is_valid_id(uint8_t id) const
{
    return std::find(
        parameters_.actuator_ids.begin(),
        parameters_.actuator_ids.end(),
        id) != parameters_.actuator_ids.end();
}