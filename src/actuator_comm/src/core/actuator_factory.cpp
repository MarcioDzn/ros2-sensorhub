#include "actuator_comm/core/actuator_factory.hpp"
#include "dynamixel_controller.hpp"

std::unique_ptr<ActuatorController> 
ActuatorFactory::createDynamixel()
{
    auto controller = std::make_unique<DynamixelController>();
    return controller;
}