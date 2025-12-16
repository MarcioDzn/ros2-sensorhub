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
    actuator_manager_->setGoalPosition(msg.id, msg.goal);
}

void ActuatorNode::load_parameters()
{
    set_parameters();
}

void ActuatorNode::set_parameters()
{
}

bool ActuatorNode::init_serial(const char* device, int baudrate)
{
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
    
    const char* device = DEVICE;
    int baudrate = BAUDRATE;
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
