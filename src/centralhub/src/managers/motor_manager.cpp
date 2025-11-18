#include "managers/motor_manager.hpp"

MotorManager::MotorManager(
    rclcpp::Node* node, 
    int motor_id
)   : node_(node), motor_id_(motor_id)
{}

int MotorManager::initComm(const char* device)
{
    if (device_.init(device) < 0)
        return false;
    return true;
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
    unsigned char motor_id = motor_id_;

    std::vector<unsigned char> packet { 
        MotorManager::PACKAGE_HEADER, 
        MotorManager::SET_TORQUE_COMMAND,
        motor_id,
        0x00,
        (
            MotorManager::PACKAGE_HEADER + 
            MotorManager::SET_TORQUE_COMMAND +
            motor_id
        ) // checksum 
    };

    if ( device_.writeData(packet) < 0 ) { return false; } 
    
    return true;
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
