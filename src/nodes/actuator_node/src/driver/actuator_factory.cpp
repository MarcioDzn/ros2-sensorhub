#include "driver/actuator_factory.hpp"
#include "driver/dynamixel/dynamixel_controller.hpp"

std::unique_ptr<ActuatorController> 
ActuatorFactory::createDynamixel()
{
    auto controller = std::make_unique<DynamixelController>();
    return controller;
}