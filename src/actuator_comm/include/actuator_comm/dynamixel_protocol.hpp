#ifndef DYNAMIXEL_PROTOCOL_HPP
#define DYNAMIXEL_PROTOCOL_HPP

#include "actuator_protocol.hpp"

class DynamixelProtocol : public ActuatorProtocol
{
    public:
        explicit DynamixelProtocol();

        std::vector<uint8_t> createPacketBase() override;
        std::vector<uint8_t> setHeader(
            std::vector<uint8_t> packet, uint8_t id, uint8_t instr) override;
        std::vector<uint8_t> setPayload(
            std::vector<uint8_t> packet, uint8_t* parameters, uint8_t parameter_size) override;
        std::vector<uint8_t> setChecksum(std::vector<uint8_t>packet) override;
};

#endif // DYNAMIXEL_PROTOCOL_HPP