#ifndef DYNAMIXEL_LINK_HPP
#define DYNAMIXEL_LINK_HPP

#include <span>
#include "actuator_link.hpp"
#include "actuator_comm/protocol/dynamixel_protocol.hpp"

#define WRITE_INSTR         0x03
#define READ_INSTR          0x02

#define ERROR_POS           4
#define RXPACKET_MAX_LEN    (250)

class DynamixelLink : public ActuatorLink
{
    public:
        DynamixelLink()
            : ActuatorLink(
                std::make_shared<DynamixelProtocol>(),
                std::make_shared<SerialHandler>()) {}

        int write1Byte(uint8_t id, uint8_t address, uint8_t data) override;
        int write2Byte(uint8_t id, uint8_t address, uint16_t data) override;
        int read1Byte(uint8_t id, uint8_t address, uint8_t& read_data) override;
        int read2Byte(uint8_t id, uint8_t address, uint16_t& read_data) override;

    private:
        int sendPacket(const std::vector<uint8_t>& packet);
        std::vector<uint8_t> getPacket(
	        uint8_t id, uint8_t instr, std::span<const uint8_t> params);
        int sendPacketAndReadStatus(
            uint8_t id, const std::vector<uint8_t>& packet, StatusPacket& status);
        int readPacket(std::array<uint8_t, RXPACKET_MAX_LEN>& packet);
        int readStatus(uint8_t id, StatusPacket& out);
};


#endif // DYNAMIXEL_LINK_HPP