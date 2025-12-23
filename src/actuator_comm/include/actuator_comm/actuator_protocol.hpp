#ifndef ACTUATOR_PROTOCOL_HPP
#define ACTUATOR_PROTOCOL_HPP

#include <cstdint>
#include <vector>

class ActuatorProtocol
{
    public:
        virtual std::vector<uint8_t> createPacketBase() = 0;
        virtual std::vector<uint8_t> setHeader(
            std::vector<uint8_t> packet, uint8_t id, uint8_t instr) = 0;
        virtual std::vector<uint8_t> setPayload(
            std::vector<uint8_t> packet, uint8_t* parameters, uint8_t parameter_size) = 0;
        virtual std::vector<uint8_t> setChecksum(std::vector<uint8_t> packet) = 0;
        virtual ~ActuatorProtocol() = default;
};


#endif // ACTUATOR_PROTOCOL_HPP