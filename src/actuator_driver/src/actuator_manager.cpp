#include "actuator_manager.hpp"

#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

#define RXPACKET_MAX_LEN    (250)

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
   	ssize_t result = serial_handler_->writeData(packet, out_size);
	free(packet);
	return result;
}

uint8_t ActuatorManager::setGoalPosition(uint8_t id, uint16_t goal_position)
{
	uint8_t lsb = goal_position & 0xFF;        // 8 bits menos significativos
	uint8_t msb = (goal_position >> 8) & 0xFF; // 8 bits mais significativos

	uint8_t out_size;
	uint8_t instruction_list[] = {0x1E, lsb, msb};
    
    auto packet = createPacket(id, 0x03, instruction_list, 3, out_size);
    ssize_t result = serial_handler_->writeData(packet, out_size);
	free(packet);
	return result;
}


uint16_t ActuatorManager::getPresentPosition(uint8_t id)
{
	uint8_t out_size;
	uint8_t instruction_list[] = {0x24, 0x02};

    auto packet = createPacket(id, 0x02, instruction_list, 2, out_size);
    ssize_t result_tx = serial_handler_->writeData(packet, out_size);
	free(packet);
	if (result_tx <= 0) return -1;

	StatusPacket status_packet;
	int result_rx = readStatus(id, status_packet);
	if (result_rx != 0) return -1;

	uint16_t value =
    	static_cast<uint16_t>(status_packet.params[0]) |
    	(static_cast<uint16_t>(status_packet.params[1]) << 8);

	return value;
}


/*
 * função inteiramente baseada em:
 * https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/jazzy/c%2B%2B/src/dynamixel_sdk/protocol1_packet_handler.cpp
*/
int ActuatorManager::readPacket(uint8_t* packet)
{
	int result = -1;

	size_t packet_size = 6; // tamanho minimo (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)
	size_t read_size = 0;
	uint8_t checksum = 0;

	auto start = std::chrono::steady_clock::now();
  	constexpr auto TIMEOUT = std::chrono::milliseconds(20);

	while(true)
	{
		if (std::chrono::steady_clock::now() - start > TIMEOUT)
      		return -2; // timeout

		ssize_t n = serial_handler_->readData(packet + read_size,
											packet_size - read_size);

		if (n <= 0)
			continue;

		read_size += static_cast<size_t>(n);

		// se já tiver pego todos os dados importantes
		// ou pelo menos a quantidade necessária
		if (read_size >= packet_size)
		{
			uint8_t i = 0;
			for (i = 0; i < (read_size-1); i++)
				if (packet[i] == 0xFF && packet[i+1] == 0xFF) 
					break;

			if (i == 0)
			{
				if (packet[PKT_ID] > 0xFD ||                  // unavailable ID
					packet[PKT_LENGTH] > RXPACKET_MAX_LEN ||  // unavailable Length
					packet[PKT_ERROR] > 0x7F)                 // unavailable Error
				{
					// remove o primeiro byte no pacote
					for (uint16_t s = 0; s < read_size - 1; s++)
						packet[s] = packet[1 + s];
					read_size -= 1;
					continue;
				}

				// recalcula o tamanho exato do pacote
				if (packet_size != packet[PKT_LENGTH] + PKT_LENGTH + 1)
				{
					packet_size = packet[PKT_LENGTH] + PKT_LENGTH + 1;
					continue;
				}

				if (read_size < packet_size)
				{
					// TODO: checar timeout
					continue;
				}

				// calcula checksum
				checksum = 0;
				for (uint16_t i = 2; i < packet_size - 1; i++)   // exceto header, checksum
					checksum += packet[i];
				checksum = ~checksum;

				// verifica checksum
				if (packet[packet_size - 1] == checksum)
				{
					result = 0;
				}
				else
				{
					result = -1;
				}
				break;
				
			} else 
			{
				for (uint8_t s = 0; s < read_size - i; s++)
				{
					packet[s] = packet[s+i];
				}
				read_size -= i;
			}
		}
	}

	return result;
}

int ActuatorManager::readStatus(uint8_t id, StatusPacket& out)
{
	uint8_t rxbuffer[RXPACKET_MAX_LEN];

	if (readPacket(rxbuffer) != 0) return -1;
	if (id != rxbuffer[PKT_ID]) return -1;

	uint8_t length = rxbuffer[PKT_LENGTH];
	uint8_t error = rxbuffer[PKT_ERROR];

	out.id = rxbuffer[PKT_ID];
	out.error = error;

	if (error != 0) return -1; 

	for (uint8_t i = 0; i < length-2; i++)
	{
		out.params[i] = rxbuffer[PKT_PARAMETER0+i];
	}

	return 0;
}

ActuatorManager::~ActuatorManager() = default;
