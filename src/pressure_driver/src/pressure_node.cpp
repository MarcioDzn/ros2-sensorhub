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

PressureNode::PressureNode(const rclcpp::NodeOptions & options) : Node("pressure_node", options)
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

void PressureNode::load_hardware_config()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();
    int active_ports = 0;
    for (const auto & [name, value] : all_params) {
        // busca configurações de devices: devices.<nome>.path
        if (name.find("devices.") == 0 && name.find(".path") != std::string::npos) {
            std::string prefix = name.substr(0, name.rfind(".path"));
            std::string path = value.get<std::string>();
            
            int baudrate = all_params.count(prefix + ".baudrate") ? 
                           all_params.at(prefix + ".baudrate").get<int>() : 115200;

            RCLCPP_INFO(this->get_logger(), "Iniciando Hardware: %s @ %d bps", path.c_str(), baudrate);

            auto handler = std::make_shared<SerialHandler>();
            
            if (handler->init(path.c_str()) < 0) {
                RCLCPP_ERROR(this->get_logger(), "FALHA AO ABRIR PORTA SERIAL: %s", path.c_str());
                continue; 
            }
            handler->setDefaultConfig();
            handler->setBaudRate(baudrate);

            hardware_map_[path] = {handler};
            active_ports++;

            RCLCPP_INFO(this->get_logger(), "Porta %s configurada com sucesso.", path.c_str());
        }
    }

    if (active_ports == 0) {
        RCLCPP_FATAL(this->get_logger(), "NENHUMA porta serial pôde ser aberta. O nó não pode funcionar.");
        throw std::runtime_error("Falha total na inicialização do hardware serial.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Inicialização concluída. %d porta(s) ativa(s).", active_ports);
    }
}

void PressureNode::load_pressure_sensors_config()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [name, value] : all_params)
    {
        if (name.find(".pressure_") != std::string::npos && name.find(".id") != std::string::npos) {
            
            std::string prefix = name.substr(0, name.rfind(".id"));
            std::string type = name.substr(0, name.find("."));

            PressureSensor press;
            press.id = value.get<int>();
            press.device  = all_params.count(prefix + ".device")  ? all_params.at(prefix + ".device").get<std::string>() : "acm0";

            pressure_sensors_[press.id] = press;
            
            RCLCPP_INFO(this->get_logger(), "Sensor de pressao ID %d carregado.", press.id);
        }
    }
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
    
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PressureNode>(options);
        
    RCLCPP_INFO(node->get_logger(), "Nó iniciado.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}