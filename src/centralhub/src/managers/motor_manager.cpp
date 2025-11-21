#include "managers/motor_manager.hpp"
#include "device_comm/packet_builder_base.hpp"
#include "device_comm/dynamixel_packet_builder.hpp"

#define DXL_LOBYTE(w) ((uint8_t)((w) & 0xFF))
#define DXL_HIBYTE(w) ((uint8_t)(((w) >> 8) & 0xFF))

#define PACKET_OVERHEAD_SIZE 7 // apenas para TESTE
#define INSTRUCTION_WRITE 3
#define TIMEOUT_MS 15

#define HEADER 0xFF

#define B1SIZE 1
#define B2SIZE 2

#define TORQUE_ADDR 0x18
#define GOAL_POSITION_ADDR

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
bool MotorManager::setTorqueCurr(int torque)
{
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[1] = { torque };
    const std::vector<uint8_t> packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(B1SIZE)
        .setAddress(TORQUE_ADDR)
        .setInstruction(INSTRUCTION_WRITE)
        .addParameter(params)
        .setHeader(HEADER)
        .setChecksum()
        .build();
    delete builder;  // libera memória

    std::vector<uint8_t> response;
    int n = sendReceivePacket(packet, response, TIMEOUT_MS);
}

bool MotorManager::setSpeed(double speed)
{
    return false;
}

bool MotorManager::setAngle(double angle)
{
    PacketBuilderBase* builder = new DynamixelPacketBuilder();
    uint8_t params[2] = { DXL_LOBYTE(angle), DXL_HIBYTE(angle) };;
    const std::vector<uint8_t> packet = builder->startPacket()
        .setID(motor_id_)
        .setParamLength(B2SIZE)
        .setAddress(GOAL_POSITION_ADDR)
        .setInstruction(INSTRUCTION_WRITE)
        .addParameter(params)
        .setHeader(HEADER)
        .setChecksum()
        .build();
    delete builder;

    std::vector<uint8_t> response;
    int n = sendReceivePacket(packet, response, TIMEOUT_MS);
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
    int timeout_ms
) {
    if (sendPacket(packet) < 0) {
        std::cerr << "Falha ao enviar pacote.\n";
        return -1;
    }
    
    return receivePacket(response, timeout_ms); // precisar ajustar receivePacket para aceitar timeout
}

int MotorManager::sendPacket(const std::vector<uint8_t>& packet)
{
    if ( device_.writeData(packet) < 0 ) { return -1; } 
    return 0;
}

int MotorManager::receivePacket(std::vector<uint8_t>& buffer, int timeout_ms)
{
    buffer.clear();
    
    // precisa de 6 bytes pra saber o tamanho total
    // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM
    size_t header_min_size = 6; 
    size_t total_expected = header_min_size;
    
    bool header_parsed = false;
    size_t total_read = 0;
    
    buffer.resize(128); 

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        int n = device_.readData(buffer.data() + total_read, buffer.size() - total_read);

        if (n > 0) {
            total_read += n;
        }

        // tenta descobrir o tamanho, após ler os 4 bytes iniciais
        if (!header_parsed && total_read >= header_min_size) {
            
            // verifica o header
            if (buffer[0] == 0xFF && buffer[1] == 0xFF) {
                
                uint8_t body_length = buffer[3]; 
                
                // tamanho total do pacote 6 bytes iniciais + body_length -1
                total_expected = 4 + body_length;
                
                // Ajusta o buffer se o pacote for maior que o reservado (128)
                if (buffer.size() < total_expected) {
                    buffer.resize(total_expected);
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
            buffer.resize(total_read); 
            return static_cast<int>(total_read);
        }

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
