#include "pressure_node.hpp"

#include <vector>
#include <sstream>

#define MAX_BUFFER_COLLECT          6 * 16
#define BUFFER_SIZE                 8 * 16 * 5
#define DEVICE                      "/dev/ttyACM0"
#define BAUDRATE                    115200

using namespace std::chrono_literals;

// TODO: criar um utils pra botar esse tipo de funcao
std::vector<uint16_t> parse_numbers_from_string(const char* buffer)
{
    std::vector<uint16_t> values;
    std::stringstream ss(buffer);
    std::string token;
    
    while (ss >> token)
    {
        values.push_back(std::stoi(token));
    } 
    
    return values;
}

PressureNode::PressureNode() : Node("pressure_node")
{
    load_parameters();
    
    // cria um publisher pra cada sensor de pressao
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    RCLCPP_INFO(this->get_logger(), "Criando publisher para o sensor de pressao");
    publisher_ = this->create_publisher<InsoleData>("/pressure", qos);
    
    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_), 
        std::bind(&PressureNode::timer_callback, this));
}

// callback que envia os dados do sensor
void PressureNode::timer_callback()
{
    auto message = InsoleData();
    
    size_t max_size = MAX_BUFFER_COLLECT;
    char buffer[BUFFER_SIZE]; // 8 bits x 16 valores x 5 caracteres pra cada valor
    get_pressure_data(buffer, max_size);
    
    auto values = parse_numbers_from_string(buffer);

    serial_handler_->clearBuffer();

    for (size_t i = 0; i < values.size(); ++i)
    {
        message.pressures.push_back(values[i]);
    }
    
    message.stamp = this->get_clock()->now();
    
    publisher_->publish(message);
}

void PressureNode::load_parameters()
{
    this->declare_parameter<int>("update_rate_ms", 15);
    set_parameters();
}

void PressureNode::set_parameters()
{
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
}

bool PressureNode::init_serial(const char* device, int baudrate)
{
    RCLCPP_INFO(this->get_logger(), "Tentando conectar com o dispositivo %s com baudrate %d", device, baudrate);
    
    serial_handler_ = std::make_unique<SerialHandler>();
    
    if ( serial_handler_->init(device) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a porta serial");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Inicializacao realizada");
    
    if ( serial_handler_->setDefaultConfig() < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao aplicar a configuracao padrao da porta serial");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Configuracao realizada");
    
    if ( serial_handler_->setBaudRate(baudrate) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao definir baudrate");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Aplicacao de baurate realizada");
    
    return true;
}

void PressureNode::get_pressure_data(char* buffer, size_t max_size)
{
    size_t i = 0;
    char c;
    
    while(i < max_size-1)
    {
        ssize_t n = serial_handler_->readData(&c, 1);
        if (n <= 0) break; // nada foi lido ou erro
        buffer[i++] = c;
        if (c == '\0') break; // chegou ao final da string
    }
    buffer[i] = '\0';
}

PressureNode::~PressureNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PressureNode>();
    
    node->declare_parameter<std::string>("device", DEVICE);
    node->declare_parameter<int>("baudrate", BAUDRATE);

    std::string device_str = node->get_parameter("device").as_string();
    const char* device = device_str.c_str();
    int baudrate = node->get_parameter("baudrate").as_int();
    if ( !node->init_serial(device, baudrate) )
    {
        RCLCPP_FATAL(node->get_logger(), "Erro ao configurar porta serial. Finalizando execucao");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "PressureNode inicializado");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
