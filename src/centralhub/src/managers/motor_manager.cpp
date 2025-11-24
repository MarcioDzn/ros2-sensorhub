#include "managers/motor_manager.hpp"
#include "device_comm/packet_builder_base.hpp"
#include "device_comm/dynamixel_packet_builder.hpp"

#define DXL_LOBYTE(w) ((uint8_t)((w) & 0xFF))
#define DXL_HIBYTE(w) ((uint8_t)(((w) >> 8) & 0xFF))

#define PACKET_OVERHEAD_SIZE 7 // apenas para TESTE
#define INSTRUCTION_READ 2
#define INSTRUCTION_WRITE 3
#define TIMEOUT_MS 15

#define HEADER 0xFF

#define B1SIZE 1
#define B2SIZE 2
#define B3SIZE 3

#define ENABLE 1

#define TORQUE_ENABLE_ADDR 0x18
#define LED_ENABLE_ADDR 0x19
#define GOAL_POSITION_ADDR 0x1E
#define PRESENT_POSITION_ADDR 0x24

MotorManager::MotorManager(
    rclcpp::Node* node, 
    int motor_id
)   : node_(node), motor_id_(motor_id)
{}

int MotorManager::initComm(const char* device, int baudrate)
{
    return device_.init(device, baudrate);
}

void MotorManager::loadParameters()
{
}

void MotorManager::createServer()
{
}

// controle
int MotorManager::enableTorque(uint8_t *error)
{
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[2] = { TORQUE_ENABLE_ADDR, ENABLE };
    const std::vector<uint8_t> packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(B2SIZE)
        .setInstruction(INSTRUCTION_WRITE)
        .addParameter(params)
        .setHeader(HEADER)
        .setChecksum()
        .build();

    std::vector<uint8_t> response;
    return sendReceivePacket(packet, response, error, TIMEOUT_MS);
}

int MotorManager::enableLED(uint8_t *error)
{
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[2] = { LED_ENABLE_ADDR, ENABLE };
    const std::vector<uint8_t> packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(B2SIZE)
        .setInstruction(INSTRUCTION_WRITE)
        .addParameter(params)
        .setHeader(HEADER)
        .setChecksum()
        .build();

    std::vector<uint8_t> response;
    return sendReceivePacket(packet, response, error, TIMEOUT_MS);
}

bool MotorManager::setSpeed(double speed)
{
    return false;
}

int MotorManager::setGoalPosition(uint16_t angle, uint8_t *error)
{
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[3] = { GOAL_POSITION_ADDR, DXL_LOBYTE(angle), DXL_HIBYTE(angle) };
    const std::vector<uint8_t> packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(B3SIZE)
        .setInstruction(INSTRUCTION_WRITE)
        .addParameter(params)
        .setHeader(HEADER)
        .setChecksum()
        .build();

    std::cout << "Pacote: ";
    for (auto byte : packet)
        std::cout << "0x" << std::hex << std::uppercase 
                  << std::setw(2) << std::setfill('0') 
                  << (int)byte << " ";
    std::cout << std::dec << std::endl; 
    delete builder;
    
    std::vector<uint8_t> response;
    return sendReceivePacket(packet, response, error, TIMEOUT_MS);
}

int MotorManager::getPresentPosition(uint8_t *error)
{
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[2] = { PRESENT_POSITION_ADDR, B2SIZE };
    const std::vector<uint8_t> packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(B2SIZE)
        .setInstruction(INSTRUCTION_READ)
        .addParameter(params)
        .setHeader(HEADER)
        .setChecksum()
        .build();

    std::vector<uint8_t> response;
    return sendReceivePacket(packet, response, error, TIMEOUT_MS);
}

// configs básicas
bool MotorManager::setRS485Baud(int baud)
{
    return false;
}

bool MotorManager::setSpinDir(SpinDirection dir)
{
    return false;
}

// limites
bool MotorManager::setMaxAngle(double angle)
{
    return false;
}

bool MotorManager::setMaxSpeed(double speed)
{
    return false;
}

bool MotorManager::setMaxAccel(int accel)
{
    return false;
}

bool MotorManager::setMaxTorqueCurr(int torque)
{
    return false;
}

bool MotorManager::setTorqueCurrRamp(int ramp)
{
    return false;
}

int MotorManager::sendReceivePacket(
    const std::vector<uint8_t>& packet, 
    std::vector<uint8_t>& response,
    uint8_t *error,
    int timeout_ms
) {
    if (sendPacket(packet) < 0) {
        std::cerr << "Falha ao enviar pacote.\n";
        return -1;
    }
    
    return receivePacket(response, error, timeout_ms); // precisar ajustar receivePacket para aceitar timeout
}

int MotorManager::sendPacket(const std::vector<uint8_t>& packet)
{
    if ( device_.writeData(packet) < 0 ) { return -1; } 
    return 0;
}

int MotorManager::receivePacket(std::vector<uint8_t>& packet, uint8_t* error, int timeout_ms)
{
    packet.clear();
    
    // precisa de 6 bytes pra saber o tamanho total
    // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM
    size_t header_min_size = 6; 
    size_t total_expected = header_min_size;
    
    bool header_parsed = false;
    size_t total_read = 0;
    
    packet.resize(128); 

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        int n = device_.readData(packet.data() + total_read, packet.size() - total_read);

        if (n > 0) {
            total_read += n;
        }

        // tenta descobrir o tamanho, após ler os 4 bytes iniciais
        if (!header_parsed && total_read >= header_min_size) {
            
            // verifica o header
            if (packet[0] == 0xFF && packet[1] == 0xFF) {
                
                uint8_t body_length = packet[3]; 
                
                // tamanho total do pacote 6 bytes iniciais + body_length -1
                total_expected = 4 + body_length;
                
                // Ajusta o packet se o pacote for maior que o reservado (128)
                if (packet.size() < total_expected) {
                    packet.resize(total_expected);
                }
                
                header_parsed = true;
            }
            else {
                std::cerr << "Erro: Header (FF FF) inválido recebido.\n";
                return -1;
            }
        }

        // tudo já foi lido e o tamanho é conhecido?
        if (header_parsed && total_read >= total_expected) {
            packet.resize(total_read); 
            return static_cast<int>(total_read);
        }

        if (error != 0)
            *error = (uint8_t)packet[DynamixelPacketBuilder::ERROR_POSITION];

        // verifica timeout
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
            
        if (elapsed >= timeout_ms) {
            return -1; 
        }

        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

// operação
bool MotorManager::applySettings()
{
    return false;
}

bool MotorManager::start()
{
    return false;
}

bool MotorManager::stop()
{
    return false;
}
