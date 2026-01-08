#ifndef DYNAMIXEL_LINK_HPP
#define DYNAMIXEL_LINK_HPP

#include <span>
#include <cstdint>
#include <vector>
#include <memory>
#include <chrono>
#include <array>

#include "dynamixel_protocol.hpp"
#include "common_serial/serial_handler.hpp"

#define WRITE_INSTR         0x03
#define READ_INSTR          0x02

#define ERROR_POS           4
#define RXPACKET_MAX_LEN    (250)

struct StatusPacket
{
    uint8_t id;
    uint8_t error;
    std::array<uint8_t, RXPACKET_MAX_LEN> params;
};

class DynamixelLink 
{
    public:
        DynamixelLink(std::shared_ptr<SerialHandler> transport, 
                      std::shared_ptr<DynamixelProtocol> protocol)
            :   protocol_(std::move(protocol)), 
                transport_(std::move(transport)) {}

        DynamixelLink() = default;

        int write1Byte(uint8_t id, uint8_t address, uint8_t data);
        int write2Byte(uint8_t id, uint8_t address, uint16_t data);
        int read1Byte(uint8_t id, uint8_t address, uint8_t& read_data);
        int read2Byte(uint8_t id, uint8_t address, uint16_t& read_data);
    
    protected:
        int readPacket(std::array<uint8_t, RXPACKET_MAX_LEN>& packet);
        int readStatus(uint8_t id, StatusPacket& out);

    private:
        int sendPacket(const std::vector<uint8_t>& packet);
        int sendPacketAndReadStatus(
            uint8_t id, const std::vector<uint8_t>& packet, StatusPacket& status);
        std::vector<uint8_t> getPacket(
	        uint8_t id, uint8_t instr, std::span<const uint8_t> params);


        std::shared_ptr<DynamixelProtocol> protocol_;
        std::shared_ptr<SerialHandler> transport_;

};


#endif // DYNAMIXEL_LINK_HPP