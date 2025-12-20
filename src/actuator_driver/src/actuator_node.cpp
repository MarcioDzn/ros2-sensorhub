#include "actuator_node.hpp"

#define DEVICE                      "/dev/ttyUSB0"
#define BAUDRATE                    2000000

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions & options) 
    : Node("actuator_node", options)
{   
    load_actuators_config();
    load_hardware_config();
    
    actuator_subscriber_ = this->create_subscription<ActuatorGoalPosition>(
        "goal_position", 10, std::bind(
        &ActuatorNode::goal_position_callback, this, 
        std::placeholders::_1));
    
    motor_service_ = this->create_service<SetMotorConfig>("motor_config", std::bind(
        &ActuatorNode::motor_service_callback, this,
        std::placeholders::_1, std::placeholders::_2));
}

void ActuatorNode::load_hardware_config()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [name, value] : all_params) {
        // procura por parametros devices.X.path
        if (name.find("devices.") == 0 && name.find(".path") != std::string::npos) {
            
            std::string prefix = name.substr(0, name.rfind(".path"));
            std::string path = value.get<std::string>();
            
            // pega baudrate ou usa padrao
            int baudrate = all_params.count(prefix + ".baudrate") ? 
                           all_params.at(prefix + ".baudrate").get<int>() : 2000000;

            RCLCPP_INFO(this->get_logger(), "Configurando Hardware: %s @ %d", path.c_str(), baudrate);

            auto handler = std::make_shared<SerialHandler>();
            if (handler->init(path.c_str()) < 0 || handler->setBaudRate(baudrate) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Falha ao abrir porta %s", path.c_str());
                continue; 
            }

            auto manager = std::make_shared<ActuatorManager>();
            manager->setSerialHandler(handler.get());

            hardware_map_[path] = {handler, manager};
        }
    }
}

void ActuatorNode::load_actuators_config()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [name, value] : all_params)
    {
        if (name.find(".actuator_") != std::string::npos && name.find(".id") != std::string::npos) {
            
            std::string prefix = name.substr(0, name.rfind(".id"));
            std::string type = name.substr(0, name.find("."));

            if (actuators_config_.find(type) == actuators_config_.end()) {
                std::string res_key = type + ".angular_resolution";
                actuators_config_[type].angular_resolution = all_params.count(res_key) ? 
                    all_params.at(res_key).get<double>() : 0.088;
            }

            Actuator act;
            act.id = value.get<int>();
            act.min_deg = all_params.count(prefix + ".min_deg") ? all_params.at(prefix + ".min_deg").get<int>() : 0;
            act.max_deg = all_params.count(prefix + ".max_deg") ? all_params.at(prefix + ".max_deg").get<int>() : 360;
            act.device  = all_params.count(prefix + ".device")  ? all_params.at(prefix + ".device").get<std::string>() : "usb0";

            actuators_config_[type].actuators[act.id] = act;
            
            RCLCPP_INFO(this->get_logger(), "Atuador: [%s] ID %d carregado.", type.c_str(), act.id);
        }
    }
}

// recebe solicitacoes
void ActuatorNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    auto command = request->command;
    auto id = request->motor_id;

    response->success = false;

    // encontra o atuador
    Actuator found_actuator;
    bool exists = false;
    
    for (auto const& [type, type_struct] : actuators_config_) {
        if (type_struct.actuators.count(request->motor_id)) {
            found_actuator = type_struct.actuators.at(request->motor_id);
            exists = true;
            break;
        }
    }

    if (!exists) {
        RCLCPP_ERROR(this->get_logger(), "Service: ID %d não encontrado", request->motor_id);
        return;
    }

    // encontra o hardware correto
    auto hw_it = hardware_map_.find(found_actuator.device);
    if (hw_it == hardware_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Service: Device %s offline", found_actuator.device.c_str());
        return;
    }

    auto& manager = hw_it->second.manager;

    // executa o comando
    RCLCPP_INFO(this->get_logger(), "Service: '%s' em ID %d", request->command.c_str(), request->motor_id);

    int result = -1;
    if (request->command == "set_goal_position" && !request->params.empty()) {
        uint16_t goal = static_cast<uint16_t>(request->params[0]);
        result = manager->setGoalPosition(request->motor_id, goal);
    } 
    else if (request->command == "enable_torque") {
        result = manager->setTorque(request->motor_id, 1);
    } 
    else if (request->command == "disable_torque") {
        result = manager->setTorque(request->motor_id, 0);
    }

    if (result == 0) response->success = true;
}

void ActuatorNode::goal_position_callback(const ActuatorGoalPosition::SharedPtr msg)
{

    if (actuators_config_.count(msg->type) == 0) {
        RCLCPP_ERROR(this->get_logger(), "Tipo %s desconhecido", msg->type.c_str());
        return;
    }

    auto& type_struct = actuators_config_[msg->type];
    if (type_struct.actuators.count(msg->id) == 0) {
        RCLCPP_ERROR(this->get_logger(), "ID %d não cadastrado no tipo %s", msg->id, msg->type.c_str());
        return;
    }

    Actuator actuator = type_struct.actuators[msg->id];

    // busca pela interface correta
    auto hw_it = hardware_map_.find(actuator.device);
    if (hw_it == hardware_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Hardware %s não inicializado para motor %d", actuator.device.c_str(), actuator.id);
        return;
    }

    auto& manager = hw_it->second.manager;
    
    if (msg->goal > actuator.max_deg || msg->goal < actuator.min_deg) {
        RCLCPP_WARN(this->get_logger(), "Comando fora dos limites: ID %d Goal %.2f", actuator.id, msg->goal);
        return;
    }

    int16_t goal_pos_native = static_cast<int16_t>(std::round(msg->goal / type_struct.angular_resolution));
    
    if (manager->setGoalPosition(actuator.id, goal_pos_native) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Erro de comunicação ao mover motor %d", actuator.id);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Motor %d movido para %.2f", actuator.id, msg->goal);
    }
}

SerialHandler* ActuatorNode::init_serial(const char* device, int baudrate)
{
    RCLCPP_INFO(this->get_logger(), "Tentando conectar com o dispositivo %s com baudrate %d", device, baudrate);
    auto handler = std::make_unique<SerialHandler>();
    
    if ( handler->init(device) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a porta serial");
        return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Inicializacao realizada");
    
    if ( handler->setDefaultConfig() < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao aplicar a configuracao padrao da porta serial");
        return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Configuracao realizada");
    
    if ( handler->setBaudRate(baudrate) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao definir baudrate");
        return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Aplicacao de baurate realizada");
    
    serial_handlers_[device] = std::move(handler);
    RCLCPP_INFO(this->get_logger(), "Inicializacao concluida");
    
    return serial_handlers_[device].get();
}

ActuatorNode::~ActuatorNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ActuatorNode>(options);
        
    RCLCPP_INFO(node->get_logger(), "Nó iniciado.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}