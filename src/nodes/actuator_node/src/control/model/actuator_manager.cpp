#include "control/model/actuator_manager.hpp"

ActuatorManager::ActuatorManager(const std::vector<uint8_t>& ids)
{
    create_actuators(ids);
}

void ActuatorManager::create_actuator(uint8_t id)
{
    if (get_actuator_by_id(id) == nullptr)
        actuators_.push_back(std::make_shared<Actuator>(id));
}

void ActuatorManager::create_actuators(const std::vector<uint8_t>& ids)
{
    actuators_.reserve(ids.size());
    for (const auto& id : ids)
        create_actuator(id);
}

int ActuatorManager::update_torque(uint8_t id, bool status)
{
    auto* actuator = get_actuator_by_id(id);
    if (!actuator) return -1;
    actuator->set_torque_status(status);
    return 0;
}

int ActuatorManager::update_position(uint8_t id, uint16_t position)
{
    auto* actuator = get_actuator_by_id(id);
    if (!actuator) return -1;
    actuator->set_position(position);
    return 0;
}

Actuator* ActuatorManager::get_actuator_by_id(uint8_t id)
{
    for (auto& actuator : actuators_)
        if (actuator->get_id() == id) 
            return actuator.get();
    return nullptr;
}