#include "managers/motor_manager.hpp"

MotorManager::MotorManager(rclcpp::Node* node, const char* device)
    : node_(node), device_()
{
    device_.init(device);
}

void MotorManager::loadParameters()
{
}

void MotorManager::createServer()
{
}

// controle
bool MotorManager::setTorqueCurr(int torque)
{
    return false;
}

bool MotorManager::setSpeed(double speed)
{
    return false;
}

bool MotorManager::setAngle(double angle)
{
    return false;
}

// configs básicas
bool MotorManager::setRS485Baud(int baud)
{
    return false;
}

bool MotorManager::setSpinDir(SpinDirection dir)
{
    return false;
}

// limites
bool MotorManager::setMaxAngle(double angle)
{
    return false;
}

bool MotorManager::setMaxSpeed(double speed)
{
    return false;
}

bool MotorManager::setMaxAccel(int accel)
{
    return false;
}

bool MotorManager::setMaxTorqueCurr(int torque)
{
    return false;
}

bool MotorManager::setTorqueCurrRamp(int ramp)
{
    return false;
}

// operação
bool MotorManager::applySettings()
{
    return false;
}

bool MotorManager::start()
{
    return false;
}

bool MotorManager::stop()
{
    return false;
}
