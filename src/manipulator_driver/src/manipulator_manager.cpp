#include "manipulator_manager.hpp"

ManipulatorManager::ManipulatorManager() {}

uint8_t* ManipulatorManager::createPacket(
	uint8_t id, 
	uint8_t instr, 
	uint8_t* parameters, 
	uint8_t parameter_size, 
	uint8_t& out_size)
{
	uint8_t packet_size = parameter_size + 2; // quantidade de parametros + checksum + instr
	out_size = packet_size + 4;
	
	uint8_t* packet = (uint8_t*) malloc(out_size); // packet_size + headers (2) + id + length
	
	
	// monta o pacote
	int i = 0;
	packet[i++] = 0xFF;
	packet[i++] = 0xFF;
	packet[i++] = id;
	packet[i++] = packet_size;
	packet[i++] = instr;
	
	// adiciona os parametros
	for (int j = 0; j<parameter_size; j++)
	{
		packet[i++] = parameters[j];
	}
	
	//checksum
	uint16_t sum = 0;
	for (int b = 2; b < (out_size) -1; b++)
	{
		sum += packet[b];
	}
	packet[i++] = ~(sum & 0xFF);
	
	return packet;
}

ManipulatorManager::~ManipulatorManager() = default;
