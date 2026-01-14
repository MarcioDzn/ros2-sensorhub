#ifndef ACTUATOR_FACTORY_HPP
#define ACTUATOR_FACTORY_HPP

#include <memory>
#include <string>

#include "driver/common/actuator_controller.hpp"

class ActuatorFactory
{
    public:
        static std::unique_ptr<ActuatorController> 
        createDynamixel();
};

#endif // ACTUATOR_FACTORY_HPP