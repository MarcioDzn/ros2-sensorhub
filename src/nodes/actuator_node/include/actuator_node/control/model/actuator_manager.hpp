#ifndef ACTUATOR_MANAGER_HPP
#define ACTUATOR_MANAGER_HPP

#include <vector>
#include <memory>

#include "model/actuator.hpp"

class ActuatorManager
{
    public:
        explicit ActuatorManager(const std::vector<uint8_t>& ids);
        virtual ~ActuatorManager() = default;

        void create_actuator(uint8_t id);
        void create_actuators(const std::vector<uint8_t>& ids);

        int update_torque(uint8_t id, bool status);
        int update_position(uint8_t id, uint16_t position);
        
        Actuator* get_actuator_by_id(uint8_t id);

        std::vector<std::shared_ptr<Actuator>>&
        get_actuators() { return actuators_; }

        const std::vector<std::shared_ptr<Actuator>>&
        get_actuators() const { return actuators_; }

    private:
        std::vector<std::shared_ptr<Actuator>> actuators_;
};

#endif // ACTUATOR_MANAGER_HPP