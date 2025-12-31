#include "pressure_node.hpp"

#include <vector>
#include <sstream>

#define MAX_BUFFER_COLLECT          6 * 16
#define BUFFER_SIZE                 8 * 16 * 5
#define DEVICE                      "/dev/ttyACM0"
#define DEFAULT_BAUDRATE                    115200

using namespace std::chrono_literals;

// TODO: criar um utils pra botar esse tipo de funcao
std::vector<uint16_t> parse_numbers_from_string(const std::string& input)
{
    std::vector<uint16_t> values;
    std::stringstream ss(input);
    std::string token;
    
    while (ss >> token) {
        try {
            values.push_back(static_cast<uint16_t>(std::stoul(token)));
        } catch (...) {
            continue;
        }
    } 
    return values;
}

PressureNode::PressureNode(const rclcpp::NodeOptions & options) : Node("pressure_node", options)
{
    load_hardware_config();
    load_pressure_sensors_config();

    load_parameters();

    create_publishers();
    
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
        if (name.find("devices.") == 0 && name.find(".path") != std::string::npos) {
            std::string prefix = name.substr(0, name.rfind(".path"));
            std::string path = value.get<std::string>();
            
            int baudrate = extract_baudrate(all_params, prefix);

            RCLCPP_INFO(this->get_logger(), "Iniciando Hardware: %s @ %d bps", path.c_str(), baudrate);

            if (setup_serial_port(path, baudrate))
            {
                RCLCPP_INFO(this->get_logger(), "Porta %s configurada com sucesso.", path.c_str());
                active_ports++;
            } else
            {
                RCLCPP_ERROR(this->get_logger(), "FALHA AO ABRIR PORTA SERIAL: %s", path.c_str());
            }
        }
    }

    if (active_ports == 0) {
        RCLCPP_FATAL(this->get_logger(), "NENHUMA porta serial pôde ser aberta. O nó não pode funcionar.");
        throw std::runtime_error("Falha total na inicialização do hardware serial.");
    }
}

bool PressureNode::setup_serial_port(const std::string &path, const int baudrate)
{
    auto handler = std::make_shared<SerialHandler>();
    
    if (handler->init(path.c_str()) < 0) {
        RCLCPP_DEBUG(this->get_logger(), "Erro de IO ao inicializar %s", path.c_str());
        return false;
    }

    handler->setDefaultConfig();
    handler->setBaudRate(baudrate);

    hardware_map_[path] = {handler};
    return true;
}

int PressureNode::extract_baudrate(
    const std::map<std::string, 
    rclcpp::ParameterValue>& params, 
    const std::string& prefix) 
{
    std::string key = prefix + ".baudrate";
    if (params.count(key)) {
        try {
            return params.at(key).get<int>();
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Baudrate inválido para %s. Usando 115200.", prefix.c_str());
        }
    }
    return DEFAULT_BAUDRATE; 
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

            // associa o path ao id
            for (auto const& [path, interface] : hardware_map_) {
                if (path.find(press.device) != std::string::npos) {
                    
                    // o código suporta um sensor por path
                    if (device_path_to_id_.count(path)) {
                        RCLCPP_WARN(this->get_logger(), 
                            "CONFLITO: O path %s já estava associado ao sensor %d. Substituindo pelo sensor %d!", 
                            path.c_str(), device_path_to_id_[path], press.id);
                    }
                    
                    device_path_to_id_[path] = press.id;
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Sensor de pressao ID %d carregado.", press.id);
        }
    }
}

void PressureNode::create_publishers()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    for (const auto &press : pressure_sensors_)
    {
        std::string topic_name = "/pressure/sensor_" + std::to_string(press.first);
        publishers_[press.first] = this->create_publisher<InsoleData>(topic_name, qos);
    }
}

void PressureNode::timer_callback()
{
    for (auto const& [path, interface] : hardware_map_) {
        char buffer[BUFFER_SIZE];
        if (get_pressure_data(interface.serial, buffer, MAX_BUFFER_COLLECT)) {
            
            auto values = parse_numbers_from_string(buffer);

            int current_sensor_id = device_path_to_id_[path];

            if (!values.empty())
            {
                publish_sensor_data(current_sensor_id, values);
            }
            
            interface.serial->clearBuffer();
        }
    }
}

bool PressureNode::get_pressure_data(std::shared_ptr<SerialHandler> handler, char* buffer, size_t max_size)
{
    size_t i = 0;
    char c;
    
    while(i < max_size-1)
    {
        ssize_t n = handler->readData(&c, 1);
        if (n <= 0) break; 
        buffer[i++] = c;
        if (c == '\0') break; 
    }
    buffer[i] = '\0';
    return (i > 0);
}

void PressureNode::publish_sensor_data(
    int sensor_id, const std::vector<uint16_t>& values)
{
    if (publishers_.count(sensor_id)) {
        auto message = InsoleData();
        message.stamp = this->get_clock()->now();
        message.pressures.assign(values.begin(), values.end());
        
        publishers_[sensor_id]->publish(message);
    }
}

void PressureNode::load_parameters()
{
    this->declare_parameter<int>("update_rate_ms", 15);
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
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