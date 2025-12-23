#ifndef DYNAMIXEL_LINK_HPP
#define DYNAMIXEL_LINK_HPP

#include "actuator_link.hpp"
#include "actuator_comm/protocol/dynamixel_protocol.hpp"

class DynamixelLink : public ActuatorLink
{
    public:
        DynamixelLink()
            : ActuatorLink(
                std::make_shared<DynamixelProtocol>(),
                std::make_shared<SerialHandler>()) {}

        int write1Byte(uint8_t id, uint8_t address, uint8_t data) override;
        int write2Byte(uint8_t id, uint8_t address, uint16_t data) override;
        int read1Byte(uint16_t& read_data) override;
        int read2Byte(uint16_t& read_data) override;
};


#endif // DYNAMIXEL_LINK_HPP