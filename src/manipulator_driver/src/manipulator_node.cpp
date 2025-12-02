#include "manipulator_node.hpp"

#define DEVICE                      "/dev/ttyUSB0"
#define BAUDRATE                    2000000

using namespace std::chrono_literals;

ManipulatorNode::ManipulatorNode() 
    : Node("manipulator_node")
{
    load_parameters();
    
    actuator_subscriber_ = this->create_subscription<ActuatorGoalPosition>(
        "goal_position", 10, std::bind(
        &ManipulatorNode::goal_position_callback, this, 
        std::placeholders::_1));
    
    motor_service_ = this->create_service<SetMotorConfig>("motor_config", std::bind(
        &ManipulatorNode::motor_service_callback, this,
        std::placeholders::_1, std::placeholders::_2));
}

// recebe solicitacoes
void ManipulatorNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    auto command = request->command;
    auto id = request->motor_id;
    RCLCPP_INFO(this->get_logger(), "%s %d", command, id);
    response->success = false;
    if (request->command == "set_goal_position"){
        uint16_t goal_pos = request->params[0];
        manipulator_manager_->setGoalPosition(request->motor_id, goal_pos);
        response->success = true;
    } else if (request->command == "enable_torque"){
        manipulator_manager_->setTorque(request->motor_id, 1);
        response->success = true;
    } else if (request->command == "disable_torque"){
        manipulator_manager_->setTorque(request->motor_id, 0);
        response->success = true;
    }
}

void ManipulatorNode::goal_position_callback(const ActuatorGoalPosition& msg)
{
    manipulator_manager_->setGoalPosition(msg.id, msg.goal);
}

void ManipulatorNode::load_parameters()
{
    set_parameters();
}

void ManipulatorNode::set_parameters()
{
}

bool ManipulatorNode::init_serial(const char* device, int baudrate)
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

void ManipulatorNode::send_packet()
{
    manipulator_manager_->setTorque(1, 1);
    manipulator_manager_->setGoalPosition(1, 1200);
}

void ManipulatorNode::set_manipulator_manager()
{
    manipulator_manager_ = std::make_unique<ManipulatorManager>();
}

ManipulatorNode::~ManipulatorNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ManipulatorNode>();
    
    const char* device = DEVICE;
    int baudrate = BAUDRATE;
    if ( !node->init_serial(device, baudrate) )
    {
        RCLCPP_FATAL(node->get_logger(), "Erro ao configurar porta serial. Finalizando execucao");
        rclcpp::shutdown();
        return 1;
    }
    
    // instancia do manipulator manager com o serial handler
    node->set_manipulator_manager();
    node->get_manipulator_manager()->setSerialHandler(node->get_serial_handler());
        
    RCLCPP_INFO(node->get_logger(), "ManipulatorNode inicializado");
    node->send_packet();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
