#include "actuator_node.hpp"

#define DEVICE                      "/dev/ttyUSB0"
#define BAUDRATE                    2000000

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions & options) 
    : Node("actuator_node", options)
{   
    load_hardware_config();
    load_actuators_config();

    actuator_subscriber_ = this->create_subscription<ActuatorGoalPosition>(
        "goal_position", 10, 
        [this](const ActuatorGoalPosition::SharedPtr msg) {
            this->goal_position_callback(msg);
        });
    
    motor_service_ = this->create_service<SetMotorConfig>("motor_config", 
        [this](const std::shared_ptr<SetMotorConfig::Request> req, std::shared_ptr<SetMotorConfig::Response> res) {
            this->motor_service_callback(req, res);
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::load_hardware_config()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();
    int active_ports = 0;
    for (const auto & [name, value] : all_params) {
        // busca configurações de devices: devices.<nome>.path
        if (name.find("devices.") == 0 && name.find(".path") != std::string::npos) {
            std::string prefix = name.substr(0, name.rfind(".path"));
            std::string path = value.get<std::string>();
            std::string path_name = prefix.substr(prefix.rfind(".") + 1);
            
            int baudrate = all_params
                .at(prefix + ".baudrate").get<int>();

            std::string actuator_type = all_params
                .at(prefix + ".actuator_type").get<std::string>();

            RCLCPP_INFO(this->get_logger(), "Iniciando Hardware: %s @ %d bps", path.c_str(), baudrate);
            
            // instanciando controller
            std::shared_ptr<ActuatorController> controller;
            if (actuator_type == "dynamixel") {
                controller = std::make_shared<DynamixelController>();
            } else {
                RCLCPP_FATAL(this->get_logger(),
                    "Tipo de actuator desconhecido: %s", actuator_type.c_str());
                throw std::runtime_error("Tipo inválido");
            }
            
            if (controller->init(path, baudrate) < 0) {
                RCLCPP_ERROR(this->get_logger(), "FALHA AO ABRIR PORTA SERIAL: %s", path.c_str());
                continue; 
            }

            // armazenando controller
            controller_map_[path_name] = controller;
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

/*
void ActuatorNode::load_hardware_config()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();
    int active_ports = 0;
    for (const auto & [name, value] : all_params) {
        // busca configurações de devices: devices.<nome>.path
        if (name.find("devices.") == 0 && name.find(".path") != std::string::npos) {
            std::string prefix = name.substr(0, name.rfind(".path"));
            std::string path = value.get<std::string>();
            
            int baudrate = all_params.count(prefix + ".baudrate") ? 
                           all_params.at(prefix + ".baudrate").get<int>() : 2000000;

            RCLCPP_INFO(this->get_logger(), "Iniciando Hardware: %s @ %d bps", path.c_str(), baudrate);

            auto handler = std::make_shared<SerialHandler>();
            
            if (handler->init(path.c_str()) < 0) {
                RCLCPP_ERROR(this->get_logger(), "FALHA AO ABRIR PORTA SERIAL: %s", path.c_str());
                continue; 
            }
            handler->setDefaultConfig();
            handler->setBaudRate(baudrate);

            // cria um manager próprio pra a porta
            auto manager = std::make_shared<ActuatorManager>();
            manager->setSerialHandler(handler);

            hardware_map_[path] = {handler, manager};
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
*/

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
                actuators_config_[type].angular_resolution = all_params
                    .at(res_key).get<double>();
            }

            Actuator act;
            act.id = value.get<int>();
            act.min_deg = all_params
                .count(prefix + ".min_deg") ? all_params.at(prefix + ".min_deg").get<int>() : 0;
            act.max_deg = all_params
                .count(prefix + ".max_deg") ? all_params.at(prefix + ".max_deg").get<int>() : 360;
            act.device  = all_params
                .at(prefix + ".device").get<std::string>();

            actuators_config_[type].actuators[act.id] = act;
            
            RCLCPP_INFO(this->get_logger(), "Atuador: [%s] ID %d carregado.", type.c_str(), act.id);
        }
    }
}

void ActuatorNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    auto command = request->command;
    response->success = false;
    
    if (actuators_config_.count(request->type) == 0) {
        RCLCPP_ERROR(this->get_logger(), "Tipo %s desconhecido", request->type.c_str());
        return;
    }

    auto& type_struct = actuators_config_[request->type];
    if (type_struct.actuators.count(request->id) == 0) {
        RCLCPP_ERROR(this->get_logger(), "ID %d não cadastrado no tipo %s", request->id, request->type.c_str());
        return;
    }

    Actuator actuator = type_struct.actuators[request->id];

    // busca pelo controller correspondente
    auto it = controller_map_.find(actuator.device);
    if (it == controller_map_.end()) {
        RCLCPP_ERROR(this->get_logger(),
            "Hardware %s não inicializado para motor %d",
            actuator.device.c_str(), actuator.id);
        return;
    }

    auto controller = it->second;

    // executa o comando
    RCLCPP_INFO(this->get_logger(), "Service: '%s' em ID %d", request->command.c_str(), request->id);

    int result = -1;
    if (request->command == "set_goal_position" && !request->params.empty()) {
        uint16_t goal = static_cast<uint16_t>(request->params[0]);
        result = controller->setGoalPosition(request->id, goal);
    } 
    else if (request->command == "enable_torque") {
        result = controller->setTorque(request->id, 1);
    } 
    else if (request->command == "disable_torque") {
        result = controller->setTorque(request->id, 0);
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

    // busca pelo controller correspondente
    auto it = controller_map_.find(actuator.device);
    if (it == controller_map_.end()) {
        RCLCPP_ERROR(this->get_logger(),
            "Hardware %s não inicializado para motor %d",
            actuator.device.c_str(), actuator.id);
        return;
    }

    auto controller = it->second;
    
    if (msg->goal > actuator.max_deg || msg->goal < actuator.min_deg) {
        RCLCPP_WARN(this->get_logger(), "Comando fora dos limites: ID %d Goal %.2f", actuator.id, msg->goal);
        return;
    }

    int16_t goal_pos_native = static_cast<int16_t>(std::round(msg->goal / type_struct.angular_resolution));
    
    if (controller->setGoalPosition(actuator.id, goal_pos_native) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Erro de comunicação ao mover motor %d", actuator.id);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Motor %d movido para %.2f", actuator.id, msg->goal);
    }
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