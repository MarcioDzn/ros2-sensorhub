#include "actuator_node.hpp"

#define DEVICE                      "/dev/ttyUSB0"
#define BAUDRATE                    2000000

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode() 
    : Node("actuator_node")
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
    RCLCPP_INFO(this->get_logger(), "%s %d", command, id);
    response->success = false;
    if (request->command == "set_goal_position"){
        uint16_t goal_pos = request->params[0];
        actuator_manager_->setGoalPosition(request->motor_id, goal_pos);
        response->success = true;
    } else if (request->command == "enable_torque"){
        actuator_manager_->setTorque(request->motor_id, 1);
        response->success = true;
    } else if (request->command == "disable_torque"){
        actuator_manager_->setTorque(request->motor_id, 0);
        response->success = true;
    }
}

void ActuatorNode::goal_position_callback(const ActuatorGoalPosition& msg)
{
    // apenas ids adicionados
    bool check_id = false;
    for(const auto &id : actuator_ids_){
        if(id == msg.id){
            check_id = true;
        }
    }

    if (!check_id) return;

    // limite de rotação
    // rotaciona o atuador n graus a partir da pos atual
    // sendo o grau positivo ou negativo
    // TODO: capturar posicao atual do motor
    if (msg.goal > max_deg_ || msg.goal < min_deg_) return;
    if (msg.goal < 0) return;

    int16_t goal_unit = static_cast<int16_t>(
        std::round(msg.goal / angular_resolution_)
    );
    actuator_manager_->setGoalPosition(msg.id, goal_unit);
}

void ActuatorNode::load_parameters()
{
    this->declare_parameter<int>("update_rate_ms", 15);
    this->declare_parameter<int>("min_deg", 100);
    this->declare_parameter<int>("max_deg", 180);
    this->declare_parameter<int>("angular_resolution", 0.088);
    this->declare_parameter<std::vector<int>>("actuator_ids", {});
    set_parameters();
}

void ActuatorNode::set_parameters()
{
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
    min_deg_ = this->get_parameter("min_deg").as_int();
    max_deg_ = this->get_parameter("max_deg").as_int();
    angular_resolution_ = this->get_parameter("max_deg").as_double();
}

bool ActuatorNode::init_serial(const char* device, int baudrate)
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
    
    auto node = std::make_shared<ActuatorNode>();

    // parametros do dispositivo serial
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
    
    // instancia do atuador manager com o serial handler
    node->set_actuator_manager();
    node->get_actuator_manager()->setSerialHandler(node->get_serial_handler());
        
    RCLCPP_INFO(node->get_logger(), "ActuatorNode inicializado");
    node->send_packet();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
