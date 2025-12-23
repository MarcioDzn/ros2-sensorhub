#include "actuator_comm/link/dynamixel_link.hpp"

#define WRITE_INSTR 0x03
#define READ_INSTR 0x02

int DynamixelLink::write1Byte(uint8_t id, uint8_t address, uint8_t data)
{
    uint8_t params[2] = {address, data};
    auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, WRITE_INSTR);
    packet = protocol_->setPayload(packet, params, 2);
    packet = protocol_->setChecksum(packet);
    
    ssize_t result = transport_->writeData(packet.data(), packet.size());
    if (result < 0) return -1;

    return static_cast<int>(result);
}

int DynamixelLink::write2Byte(uint8_t id, uint8_t address, uint16_t data)
{
    uint8_t lsb = data & 0xFF;        // 8 bits menos significativos
	uint8_t msb = (data >> 8) & 0xFF; // 8 bits mais significativos
    
    uint8_t params[3] = {address, lsb, msb};
    auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, WRITE_INSTR);
    packet = protocol_->setPayload(packet, params, 3);
    packet = protocol_->setChecksum(packet);
    
    ssize_t result = transport_->writeData(packet.data(), packet.size());
    if (result < 0) return -1;
    
    return static_cast<int>(result);
}

int DynamixelLink::read1Byte(uint8_t id, uint8_t address, uint8_t& read_data)
{
    uint8_t params[2] = {address, 1};
    auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, READ_INSTR);
    packet = protocol_->setPayload(packet, params, 2);
    packet = protocol_->setChecksum(packet);
    
    ssize_t result = transport_->writeData(packet.data(), packet.size());
    if (result < 0) return -1;
    
    return static_cast<int>(result);
}