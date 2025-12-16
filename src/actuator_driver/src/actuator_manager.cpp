#include "actuator_manager.hpp"

ActuatorManager::ActuatorManager() {}


void ActuatorManager::setSerialHandler(std::shared_ptr<SerialHandler> serial) {
	serial_handler_ = serial;
}
    
uint8_t* ActuatorManager::createPacket(
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

uint8_t ActuatorManager::setTorque(uint8_t id, uint8_t status)
{
	if (status != 0 && status != 1) return -1;
	
	uint8_t out_size;
	uint8_t instruction_list[] = {0x18, status};
    
    auto packet = createPacket(id, 0x03, instruction_list, 2, out_size);
    return serial_handler_->writeData(packet, out_size);
}

uint8_t ActuatorManager::setGoalPosition(uint8_t id, uint16_t goal_position)
{
	uint8_t lsb = goal_position & 0xFF;        // 8 bits menos significativos
	uint8_t msb = (goal_position >> 8) & 0xFF; // 8 bits mais significativos

	uint8_t out_size;
	uint8_t instruction_list[] = {0x1E, lsb, msb};
    
    auto packet = createPacket(id, 0x03, instruction_list, 3, out_size);
    return serial_handler_->writeData(packet, out_size);
}



ActuatorManager::~ActuatorManager() = default;
