#include "actuator_comm/protocol/dynamixel_protocol.hpp"

#define PACKET_BASE_SIZE        5
#define MIN_PAYLOAD_SIZE        2

#define PREAMBLE_POS            0
#define ID_POS                  2
#define LENGTH_POS              3
#define INSTRUCTION_POS         4
#define PARAMETER_POS           5


DynamixelProtocol::DynamixelProtocol() {}

std::vector<uint8_t> DynamixelProtocol::createPacketBase()
{
	std::vector<uint8_t> packet(PACKET_BASE_SIZE);
	
	packet[PREAMBLE_POS]            = 0xFF;
	packet[PREAMBLE_POS+1]          = 0xFF;
    packet[LENGTH_POS]              = MIN_PAYLOAD_SIZE; // instrucao + checksum

    return packet;
}

std::vector<uint8_t> DynamixelProtocol::setHeader(
    std::vector<uint8_t> packet, uint8_t id, uint8_t instr)
{
	packet[ID_POS]                  = id;
	packet[INSTRUCTION_POS]         = instr;

    return packet;
}

std::vector<uint8_t> DynamixelProtocol::setPayload(
    std::vector<uint8_t> packet, uint8_t* parameters, uint8_t parameter_size)
{
    packet.resize(PARAMETER_POS+parameter_size);

    uint8_t payload_length = packet[LENGTH_POS];
	for (int i = 0; i<parameter_size; i++)
	{
		packet[PARAMETER_POS+i]     = parameters[i];
        payload_length++;
	}
    packet[LENGTH_POS]              = payload_length;
    return packet;
}

std::vector<uint8_t> DynamixelProtocol::setChecksum(
    std::vector<uint8_t> packet)
{
    packet.resize(packet.size()+1);

	uint16_t sum = 0;
	for (uint8_t i = ID_POS; i < packet.size()-1; i++)
	{
		sum += packet[i];
	}
	packet[packet.size()-1] = ~(sum & 0xFF);
    return packet;
}