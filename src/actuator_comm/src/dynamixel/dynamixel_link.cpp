#include "dynamixel_link.hpp"

int DynamixelLink::write1Byte(uint8_t id, uint8_t address, uint8_t data)
{
    uint8_t params[2] = {address, data};
    std::vector<uint8_t> packet = getPacket(id, WRITE_INSTR, params);
    
	StatusPacket status;
    return sendPacketAndReadStatus(id, packet, status);
}

int DynamixelLink::write2Byte(uint8_t id, uint8_t address, uint16_t data)
{
    uint8_t lsb = data & 0xFF;        // 8 bits menos significativos
	uint8_t msb = (data >> 8) & 0xFF; // 8 bits mais significativos
    
    uint8_t params[3] = {address, lsb, msb};
	std::vector<uint8_t> packet = getPacket(id, WRITE_INSTR, params);
    
	StatusPacket status;
    return sendPacketAndReadStatus(id, packet, status);
}

int DynamixelLink::read1Byte(uint8_t id, uint8_t address, uint8_t& read_data)
{
    uint8_t params[2] = {address, 1};
    std::vector<uint8_t> packet = getPacket(id, READ_INSTR, params);
    
    if (transport_->writeData(packet.data(), packet.size()) <= 0) return -1;

    StatusPacket status;
	int result = sendPacketAndReadStatus(id, packet, status);
    
    if (result == 0) {
        read_data = status.params[0];
    }
    return result;
}

int DynamixelLink::read2Byte(uint8_t id, uint8_t address, uint16_t& read_data)
{
    uint8_t params[2] = {address, 2};
	std::vector<uint8_t> packet = getPacket(id, READ_INSTR, params);
    
    if (transport_->writeData(packet.data(), packet.size()) <= 0) return -1;

	StatusPacket status;
    int result = sendPacketAndReadStatus(id, packet, status);
    
    if (result == 0) {
        read_data = static_cast<uint16_t>(status.params[0]) |
                   (static_cast<uint16_t>(status.params[1]) << 8);
    }
    return result;
}

std::vector<uint8_t> DynamixelLink::getPacket(
	uint8_t id, uint8_t instr, std::span<const uint8_t> params)
{
	auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, instr);
    packet = protocol_->setPayload(packet, params);
    packet = protocol_->setChecksum(packet);

	return packet;
}

int DynamixelLink::sendPacket(const std::vector<uint8_t>& packet)
{
	if (transport_->writeData(packet.data(), packet.size()) <= 0) 
		return -1;
	return 0;
}

int DynamixelLink::sendPacketAndReadStatus(
	uint8_t id, const std::vector<uint8_t>& packet, StatusPacket& status)
{
	if (sendPacket(packet) != 0)
		return -1;

	if (id == 0xFE)
		return 0; // broadcast não tem status

    return readStatus(id, status);
}

int DynamixelLink::readPacket(std::array<uint8_t, RXPACKET_MAX_LEN>& packet)
{
	size_t read_size = 0;
	size_t wait_length = 6; // tamanho minimo (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

	auto start = std::chrono::steady_clock::now();
	constexpr auto TIMEOUT = std::chrono::milliseconds(20);

	while (true)
	{
		if (std::chrono::steady_clock::now() - start > TIMEOUT)
			return -2;

		ssize_t n = transport_->readData(packet.data() + read_size,
			wait_length - read_size);

		if (n > 0)
			read_size += static_cast<size_t>(n);

		// se ainda não foi lido 6 bytes, volta a ler
		if (read_size < wait_length)
			continue;

		// encontra a posição do header no pacote
		uint8_t idx = 0;
		for (idx = 0; idx < (read_size - 1); idx++)
		{
			if (packet[idx] == 0xFF && packet[idx + 1] == 0xFF)
				break;
		}
		
		// se o header estiver no inicio
		if (idx == 0)
		{
			if (packet[2] > 0xFD ||
				packet[3] > RXPACKET_MAX_LEN ||
				packet[4] > 0x7F)
			{
				std::memmove(packet.data(), packet.data() + 1, read_size - 1);
				read_size -= 1;
				wait_length = 6; // reseta para busca de header
				continue;
			}

			// recalcula o tamanho real do pacote
			// para comportar os parâmetros
			size_t total_expected = packet[3] + 4;

			if (wait_length != total_expected)
			{
				wait_length = total_expected;
				// volta para o loop pra ler o resto dos parâmetros
				continue; 
			}

			// verifica o checksum
			uint8_t checksum = 0;
			for (size_t i = 2; i < wait_length - 1; i++)
				checksum += packet[i];
			checksum = ~checksum;

			// verifica se o checksum bate
			std::cout << "DEBUG - ID: " << (int)packet[2] << " LEN: " << (int)packet[3] << std::endl;
			std::cout << "DEBUG - Calculado: " << std::hex << (int)checksum 
          	<< " Recebido: " << (int)packet[wait_length - 1] << std::dec << std::endl;
			if (packet[wait_length - 1] == checksum)
				return 0;
			else	
				return -1;
		} else 
		{
			// move os dados, fazendo o header ficar no inicio
			std::memmove(packet.data(), packet.data() + idx, read_size - idx);
			read_size -= idx;
		}
	}
}

int DynamixelLink::readStatus(uint8_t id, StatusPacket& out)
{
	std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

	if (readPacket(rxbuffer) != 0) return -1;
	if (id != rxbuffer[ID_POS]) return -1;

	uint8_t length = rxbuffer[LENGTH_POS];
	uint8_t error = rxbuffer[ERROR_POS];

	out.id = rxbuffer[ID_POS];
	out.error = error;

	if (error != 0) return -1; 

	for (uint8_t i = 0; i < length-2; i++)
	{
		out.params[i] = rxbuffer[PARAMETER_POS+i];
	}

	return 0;
}