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

int ActuatorManager::update_actuator(const Actuator& updated_actuator)
{
    for (auto& actuator : actuators_)
    {
        if (actuator->get_id() == updated_actuator.get_id())
        {
            actuator->set_position(updated_actuator.get_position());
            return 0;
        }
    }
    return -1;
}

int ActuatorManager::update_actuators(const std::vector<Actuator>& updated_actuators)
{
    int success_count = 0;
    for (const auto& new_actuator : updated_actuators)
    {
        if (update_actuator(new_actuator) == 0)
            success_count++;
    }
    return (success_count == updated_actuators.size()) ? 0 : -1;
}

Actuator* ActuatorManager::get_actuator_by_id(uint8_t id)
{
    for (auto& actuator : actuators_)
        if (actuator->get_id() == id) 
            return actuator.get();
    return nullptr;
}