#include "driver/pressure_factory.hpp"

std::unique_ptr<PressureDriver> 
PressureFactory::createPressure()
{
    auto controller = std::make_unique<PressureDriver>();
    return controller;
}