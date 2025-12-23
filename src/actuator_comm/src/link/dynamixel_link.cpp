#include "actuator_comm/link/dynamixel_link.hpp"

#define WRITE_INSTR 0x03

int DynamixelLink::write1Byte(uint8_t id, uint8_t data)
{
    uint8_t params[1] = {data};
    auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, WRITE_INSTR);
    packet = protocol_->setPayload(packet, params, 1);
    packet = protocol_->setChecksum(packet);
    
    ssize_t result = transport_->writeData(packet.data(), packet.size());
    if (result < 0) return -1;
    
    return static_cast<int>(result);
}