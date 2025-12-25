#include "actuator_comm/link/dynamixel_link.hpp"

int DynamixelLink::write1Byte(uint8_t id, uint8_t address, uint8_t data)
{
    uint8_t params[2] = {address, data};
    auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, WRITE_INSTR);
    packet = protocol_->setPayload(packet, params);
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
    packet = protocol_->setPayload(packet, params);
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
    packet = protocol_->setPayload(packet, params);
    packet = protocol_->setChecksum(packet);
    
    if (transport_->writeData(packet.data(), packet.size()) <= 0) return -1;

    StatusPacket status;
    if (readStatus(id, status) != 0) return -1;

    read_data = status.params[0]; 
    return 0;
}

int DynamixelLink::read2Byte(uint8_t id, uint8_t address, uint16_t& read_data)
{
    uint8_t params[2] = {address, 2};
    auto packet = protocol_->createPacketBase();
    packet = protocol_->setHeader(packet, id, READ_INSTR);
    packet = protocol_->setPayload(packet, params);
    packet = protocol_->setChecksum(packet);
    
    if (transport_->writeData(packet.data(), packet.size()) <= 0) return -1;

    StatusPacket status;
    if (readStatus(id, status) != 0) return -1;

    read_data = static_cast<uint16_t>(status.params[0]) |
    	(static_cast<uint16_t>(status.params[1]) << 8); 
    return 0;
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

		ssize_t n = transport_->readData(packet.data() + read_size,
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
				if (packet[ID_POS] > 0xFD ||                  // unavailable ID
					packet[LENGTH_POS] > RXPACKET_MAX_LEN ||  // unavailable Length
					packet[ERROR_POS] > 0x7F)                 // unavailable Error
				{
					// remove o primeiro byte no pacote
					for (uint16_t s = 0; s < read_size - 1; s++)
						packet[s] = packet[1 + s];
					read_size -= 1;
					continue;
				}

				// recalcula o tamanho exato do pacote
				if (packet_size != static_cast<size_t>(packet[LENGTH_POS] + LENGTH_POS + 1))
				{
					packet_size = packet[LENGTH_POS] + LENGTH_POS + 1;
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
				{result = 0;}
				else
				{result = -1;}
				break;
				
			} else 
			{
				for (uint8_t s = 0; s < read_size - i; s++)
				{packet[s] = packet[s+i];}
				read_size -= i;
			}
		}
	}

	return result;
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