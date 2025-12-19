#include "actuator_node.hpp"

#define DEVICE                      "/dev/ttyUSB0"
#define BAUDRATE                    2000000

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions & options) 
    : Node("actuator_node", options)
{   
    load_parameters();
    
    actuator_subscriber_ = this->create_subscription<ActuatorGoalPosition>(
        "goal_position", 10, std::bind(
        &ActuatorNode::goal_position_callback, this, 
        std::placeholders::_1));
    
    motor_service_ = this->create_service<SetMotorConfig>("motor_config", std::bind(
        &ActuatorNode::motor_service_callback, this,
        std::placeholders::_1, std::placeholders::_2));
}

// recebe solicitacoes
void ActuatorNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    auto command = request->command;
    auto id = request->motor_id;
    response->success = false;

    Actuator found_actuator;
    bool actuator_exists = false;
    for (auto const& [type, type_struct] : actuators_) {
        if (type_struct.actuators.count(id)) {
            found_actuator = type_struct.actuators.at(id);
            actuator_exists = true;
            break;
        }
    }

    if (!actuator_exists) {
        RCLCPP_ERROR(this->get_logger(), "SERVICO: ATUADOR DE ID %d NAO ENCONTRADO NO YAML", id);
        return;
    }

    // troca o serial_handler a depender do path do device
    bool handler_set = false;
    for (auto const& [path, handler] : serial_handlers_) {
        if (path.find(found_actuator.device) != std::string::npos) {
            actuator_manager_->setSerialHandler(handler);
            handler_set = true;
            break;
        }
    }

    if (!handler_set) {
        RCLCPP_ERROR(this->get_logger(), "SERVICO: SERIAL PARA O DISPOSITIVO %s NAO ENCONTRADA", found_actuator.device.c_str());
        return;
    }

    // executa o comando
    RCLCPP_INFO(this->get_logger(), "Executando '%s' no Atuador %d (%s)", command.c_str(), id, found_actuator.device.c_str());

    if (command == "set_goal_position") {
        uint16_t goal_pos = static_cast<uint16_t>(request->params[0]);
        if (actuator_manager_->setGoalPosition(id, goal_pos) == 0) response->success = true;
    } 
    else if (command == "enable_torque") {
        if (actuator_manager_->setTorque(id, 1) == 0) response->success = true;
    } 
    else if (command == "disable_torque") {
        if (actuator_manager_->setTorque(id, 0) == 0) response->success = true;
    }
}

void ActuatorNode::goal_position_callback(const ActuatorGoalPosition& msg)
{
    // busca o tipo do atuador
    ActuatorType actuator_type;
    try {
        actuator_type = actuators_.at(msg.type);
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(this->get_logger(), "TIPO %s NAO CADASTRADO", msg.type.c_str());
        return;
    }

    // busca atuador pelo id
    Actuator actuator;
    try {
        actuator = actuator_type.actuators.at(msg.id);
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(this->get_logger(), "ID %d NAO CADASTRADO NO TIPO %s", msg.id, msg.type.c_str());
        return;
    }

    // troca o serial_handler a depender do path do device
    bool handler_set = false;
    for (auto const& [path, handler] : serial_handlers_) {
        if (path.find(actuator.device) != std::string::npos) {
            actuator_manager_->setSerialHandler(handler);
            handler_set = true;
            break;
        }
    }

    if (!handler_set) {
        RCLCPP_ERROR(this->get_logger(), "SERIAL HANDLER PARA O DISPOSITIVO %s NAO ENCONTRADO", actuator.device.c_str());
        return;
    }
    // -------------------------------------------------------

    double goal_pos_deg = msg.goal;
    if (goal_pos_deg > actuator.max_deg || goal_pos_deg < actuator.min_deg) {
        RCLCPP_WARN(this->get_logger(), 
            "COMANDO ABSOLUTO PARA ATUADOR %d FORA DOS LIMITES (%d a %d): %.2f", 
            actuator.id, actuator.min_deg, actuator.max_deg, goal_pos_deg);
        return;
    }

    // converte de graus pra a unidade do dynamixel
    int16_t goal_pos = static_cast<int16_t>(std::round(goal_pos_deg / actuator_type.angular_resolution));
    
    if (actuator_manager_->setGoalPosition(actuator.id, goal_pos) != 0) {
        RCLCPP_ERROR(this->get_logger(), "ERRO AO ENVIAR POSICAO ABSOLUTA PARA MOTOR %d", actuator.id);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Motor %d movido para %.2f deg (%d steps)", 
            actuator.id, goal_pos_deg, goal_pos);
    }
}

void ActuatorNode::load_parameters()
{
    auto all_params = this->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [name, value] : all_params)
    {
        if (name.find(".actuator_") != std::string::npos && name.find(".id") != std::string::npos) {
            
            std::string prefix = name.substr(0, name.rfind(".id"));
            std::string type = name.substr(0, name.find("."));

            if (actuators_.find(type) == actuators_.end()) {
                std::string res_key = type + ".angular_resolution";
                actuators_[type].angular_resolution = all_params.count(res_key) ? 
                    all_params.at(res_key).get<double>() : 0.088;
            }

            Actuator act;
            act.id = value.get<int>();
            act.min_deg = all_params.count(prefix + ".min_deg") ? all_params.at(prefix + ".min_deg").get<int>() : 0;
            act.max_deg = all_params.count(prefix + ".max_deg") ? all_params.at(prefix + ".max_deg").get<int>() : 360;
            act.device  = all_params.count(prefix + ".device")  ? all_params.at(prefix + ".device").get<std::string>() : "usb0";

            actuators_[type].actuators[act.id] = act;
            
            RCLCPP_INFO(this->get_logger(), "Atuador: [%s] ID %d carregado.", type.c_str(), act.id);
        }
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

void ActuatorNode::send_packet()
{
    actuator_manager_->setTorque(1, 1);
    actuator_manager_->setGoalPosition(1, 1200);
}

void ActuatorNode::set_actuator_manager()
{
    actuator_manager_ = std::make_unique<ActuatorManager>();
}

ActuatorNode::~ActuatorNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ActuatorNode>(options);

    auto all_params = node->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [name, value] : all_params) {
        if (name.find("devices.") == 0 && name.find(".path") != std::string::npos) {
            std::string prefix = name.substr(0, name.rfind(".path"));
            std::string path = value.get<std::string>();
            
            int baudrate = all_params.count(prefix + ".baudrate") ? 
                           all_params.at(prefix + ".baudrate").get<int>() : 2000000;

            if (node->init_serial(path.c_str(), baudrate) == nullptr) {
                RCLCPP_FATAL(node->get_logger(), "Falha fatal na porta %s", path.c_str());
                return 1;
            }
        }
    }
    
    node->set_actuator_manager();
    
    // Vincula a serial ao manager
    if (!node->get_serial_handlers().empty()) {
        node->get_actuator_manager()->setSerialHandler(node->get_serial_handlers().begin()->second);
    }
        
    RCLCPP_INFO(node->get_logger(), "Nó iniciado.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}