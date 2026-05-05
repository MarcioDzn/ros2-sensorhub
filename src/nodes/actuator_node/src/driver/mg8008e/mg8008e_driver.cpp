#include "driver/mg8008e/mg8008e_driver.hpp"
#include "common_serial/serial_handler.hpp"

#include <chrono>
#include <cstring>

MG8008EDriver::MG8008EDriver() {}

// Inicia comunicação com a porta serial
int MG8008EDriver::init(std::string device, int baudrate)
{
    transport_ = std::make_unique<SerialHandler>();
    if (transport_->init(device.c_str()) < 0)
        return -1; 
    transport_->setDefaultConfig();
    transport_->setBaudRate(baudrate);

    return 0;
}

std::vector<uint8_t> get4bytes(uint32_t value) {
	uint8_t b1 = (value >> 24) 	& 0xFF;
	uint8_t b2 = (value >> 16) 	& 0xFF;
	uint8_t b3 = (value >> 8) 	& 0xFF;
	uint8_t b4 = value 			& 0xFF;

	return {b4, b3, b2, b1};
}

int MG8008EDriver::set_goal_position(uint8_t id, int32_t goal_position, int32_t speed)
{
	// valores de posição
	int32_t scaled_goal_pos = goal_position * 100;
	std::vector<uint8_t> b_goal = get4bytes((uint32_t)scaled_goal_pos);

	// valores de velocidade
	int32_t scaled_speed = speed * 100;
	std::vector<uint8_t> b_speed = get4bytes((uint32_t)scaled_speed);
	
	// verifica se o valor é negativo e adiciona o padding correspondente
	uint8_t sig = (goal_position < 0) ? 0xFF : 0x00;

	// constrói os parâmetros
	std::vector<uint8_t> params = {
		b_goal[0], b_goal[1], b_goal[2], b_goal[3],
		sig, 		sig, 		sig, 		sig,
		b_speed[0], b_speed[1], b_speed[2], b_speed[3]
	};

    std::vector<uint8_t> packet = get_packet(
		id, MULTI_LOOP_2, MULTI_LOOP_2_TYPE, params);

    if (transport_->writeData(packet.data(), packet.size()) < 0) 
		return -1;

    // le o status pra evitar erros
    StatusPacket status;
    if (read_status(id, status) < 0)
        return -1;
    
    return 0;
}

int MG8008EDriver::get_current_position(uint8_t id, uint16_t& current_position)
{
    std::vector<uint8_t> packet = get_packet(
		id, READ_MULTI_LOOP_2, 0x00, {});

    if (transport_->writeData(packet.data(), packet.size()) < 0) 
		return -1;

    StatusPacket status;
    if (read_status(id, status) < 0)
        return -1;

	// TODO: Consertar aqui
	// Mudar tipo
	// verificar se pega o dado assim msm
    current_position = static_cast<uint16_t>(status.params[0]) |
                   (static_cast<uint16_t>(status.params[1]) << 8);

    return 0;
}

std::vector<uint8_t> MG8008EDriver::get_packet(
    const uint8_t id, 
	const uint16_t command, 
	const uint16_t frame_type,
	const std::vector<uint8_t>& params)
{
	size_t base_size = 5 + params.size(); 
    if (frame_type != 0x00) base_size++;

    std::vector<uint8_t> packet(base_size);

    packet[HEADER_POS]            	= 0x3E;
	packet[COMMAND_POS]         	= command;
    packet[ID_POS]                  = id;
    packet[LENGTH_POS]         		= params.size();
    
	uint8_t offset = 4;
    if (frame_type != 0x00) 
    {
        packet[offset++] = 0xEF; // FRAME_TYPE_POS
    }

	for (size_t i = 0; i < params.size(); i++) {
        packet[offset + i] = params[i];
    }

    // calculo do checksum
    uint8_t checksum = 0;
	for (size_t i = 0; i < packet.size() - 1; i++) {
		checksum += packet[i];
	}

	packet[packet.size() - 1] = checksum;

    return packet;
}

// monta pacote de dados que é enviado pelo mg8008e
int MG8008EDriver::read_packet(std::array<uint8_t, RXPACKET_MAX_LEN>& packet)
{
	size_t read_size = 0;
	size_t wait_length = 6; // tamanho minimo (HEADER COMMAND ID LENGTH FRAME_TYPE CHKSUM)

	auto start = std::chrono::steady_clock::now();
	constexpr auto TIMEOUT = std::chrono::milliseconds(20);

	while (true)
	{
		if (std::chrono::steady_clock::now() - start > TIMEOUT)
			return -2;

		ssize_t n = transport_->readData(
			packet.data() + read_size,
			wait_length - read_size);

		if (n > 0)
			read_size += static_cast<size_t>(n);

		// se ainda não foi recebido o mínimo, 
		// volta a ler
		if (read_size < wait_length)
			continue;

		// encontra a posição do header no pacote
		size_t idx = 0;
		bool found_header = false;
		for (idx = 0; idx < read_size; idx++)
		{
			if (packet[idx] == 0x3E)
			{
				found_header = true;
				break;
			}
		}

		// não encontrou header
        if (!found_header)
        {
            read_size = 0;
            wait_length = 6;
            continue;
        }

		// move o header para o início
		if (idx != 0)
        {
            std::memmove(
                packet.data(),
                packet.data() + idx,
                read_size - idx);

            read_size -= idx;
        }

		// ainda não tem bytes suficientes
        // após mover o header
        if (read_size < 6)
            continue;

		// tamanho total esperado do pacote
		// tamanho do payload + encapsulamento
		size_t total_expected = packet[3] + 6;

		// tamanho inválido
		if (total_expected > RXPACKET_MAX_LEN)
        {
            // descarta header inválido
            std::memmove(
                packet.data(),
                packet.data() + 1,
                read_size - 1);

            read_size -= 1;
            wait_length = 6;

            continue;
        }

		wait_length = total_expected;

        // ainda falta receber bytes
        if (read_size < wait_length)
            continue;

		// calculo do checksum
		uint8_t checksum = 0;

        for (size_t i = 0; i < wait_length - 1; i++)
        {
            checksum += packet[i];
        }

        // checksum OK
        if (packet[wait_length - 1] == checksum)
            return 0;

        // checksum inválido
        return -1;
	}
}

// Monta um status packet com valores úteis
// advindos do pacote montado por read_packet()
int MG8008EDriver::read_status(uint8_t id, StatusPacket& out)
{
	std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

	if (read_packet(rxbuffer) != 0) return -1;
	if (id != rxbuffer[ID_POS]) return -1;

	uint8_t length = rxbuffer[LENGTH_POS];
	uint8_t error = rxbuffer[ERROR_POS];

	out.id = rxbuffer[ID_POS];
	out.error = error;

	if (error != 0) return -1; 

    // adiciona os parâmetros
	for (uint8_t i = 0; i < length-2; i++)
	{
		out.params[i] = rxbuffer[PARAMETER_POS+i];
	}

	return 0;
}
