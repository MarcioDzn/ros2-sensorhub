#ifndef DYNAMIXEL_PROTOCOL_HPP
#define DYNAMIXEL_PROTOCOL_HPP

#include <span>
#include <cstdint>
#include <vector>

#define PACKET_BASE_SIZE        5
#define MIN_PAYLOAD_SIZE        2

#define PREAMBLE_POS            0
#define ID_POS                  2
#define LENGTH_POS              3
#define INSTRUCTION_POS         4
#define PARAMETER_POS           5

class DynamixelProtocol
{
    public:
        explicit DynamixelProtocol();

        std::vector<uint8_t> createPacketBase();
        std::vector<uint8_t> setHeader(
            std::vector<uint8_t> packet, uint8_t id, uint8_t instr);
        std::vector<uint8_t> setPayload(
            std::vector<uint8_t> packet, std::span<const uint8_t> parameters);
        std::vector<uint8_t> setChecksum(std::vector<uint8_t>packet);
};

#endif // DYNAMIXEL_PROTOCOL_HPP