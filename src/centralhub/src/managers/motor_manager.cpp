#include "managers/motor_manager.hpp"
#include "device_comm/packet_builder_base.hpp"
#include "device_comm/dynamixel_packet_builder.hpp"

#define PACKET_OVERHEAD_SIZE 7 // apenas para TESTE
#define INSTRUCTION_WRITE 3

MotorManager::MotorManager(
    rclcpp::Node* node, 
    int motor_id
)   : node_(node), motor_id_(motor_id)
{}

int MotorManager::initComm(const char* device)
{
    return device_.init(device);
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
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[1] = { torque };
    const std::vector<uint8_t>& packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(1)
        .setAddress(0x18)
        .setInstruction(INSTRUCTION_WRITE)
        .addParameter(params)
        .setHeader(0xFF)
        .setChecksum()
        .build();

    size_t packet_size = PACKET_OVERHEAD_SIZE + 2; // 7 + param_length
    std::cout << "Packet: ";
    for (size_t i = 0; i < packet_size; ++i) {
        printf("%02X ", packet[i]);
    }
    std::cout << std::endl;

    delete builder;  // libera memória

    if ( device_.writeData(packet) < 0 ) { return -1; } 

    return 0; 
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
