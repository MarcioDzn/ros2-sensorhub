#include "node/actuator_node.hpp"

#include "control/node/parameter_manager.hpp"

#include "driver/actuator_factory.hpp"

#include <vector>

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions& options) 
    : Node("actuator_node", options)
{   
    timing_log_.open("tempos_atuadores.txt", std::ios::out | std::ios::trunc);

    parameter_manager_ = std::make_shared<ParameterManager>(this);
    actuator_driver_ = ActuatorFactory::createDynamixel();

    // inicializa porta serial inserida nos parâmetros do yaml
    auto init_response = actuator_driver_->init(
        parameter_manager_->get_usb_port(), 
        parameter_manager_->get_baudrate());
    if (init_response != 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial na porta %s.", 
            parameter_manager_->get_usb_port().c_str());
        throw std::runtime_error("Falha ao inicializar ActuatorNode");
    }

    // TODO: verificar se os ids fornecidos pelo yaml
    // são de atuadores realmente conectados

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    // recebe dados de comando contínuo. Ex: posição alvo
    actuator_subscriber_ = this->create_subscription<ActuatorCommand>(
         "actuator/command", qos, 
        [this](const ActuatorCommand::SharedPtr msg) {
            
            this->set_goal_position(msg);
        });
    
    // recebe dados de comando pontual. Ex: habilitar/desabilitar torque
    set_torque_service_ = this->create_service<SetTorque>(
         "actuator/set_torque", 
        [this](const std::shared_ptr<SetTorque::Request> req, 
                std::shared_ptr<SetTorque::Response> res) {
            this->set_torque(req, res);
        });

    // envia dados de estado. Ex: posição atual
    state_publisher_ = this->create_publisher<ActuatorState>(
         "actuator/state", qos);

    // envia dados de tempo
    time_publisher_ = this->create_publisher<Time>(
         "actuator/time", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        [this]() {
            publish_position_data();
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::set_torque(
    const std::shared_ptr<SetTorque::Request> request,
    std::shared_ptr<SetTorque::Response> response)
{
    std::lock_guard<std::mutex> lock(driver_mutex_);

    // busca o atuador referente ao id
    auto id = parameter_manager_->get_id_by_name(request->name);
    if (id == -1){
        RCLCPP_WARN(this->get_logger(), "Atuador %s não encontrado", 
            request->name.c_str());
        response->success = false;
        return;
    }

    // define o torque
    auto result = actuator_driver_->set_torque(
        static_cast<uint8_t>(id), 
        request->status ? 1 : 0);
    if (result != 0) {
        RCLCPP_WARN(this->get_logger(), 
            "Falha na configuração do torque. Erro: %d", 
            static_cast<int>(result));
        response->success = false;
        return; 
    }
    
    response->success = true;
}

void ActuatorNode::publish_position_data()
{
    Time time_msg;
    ActuatorState msg;

    const auto& ids = parameter_manager_->get_ids();
    const auto& names = parameter_manager_->get_names();

    std::vector<std::string> successful_names;
    std::vector<int16_t> successful_positions;

    // ====== COMEÇO DA CONTAGEM TOTAL =======
    auto start_total = std::chrono::high_resolution_clock::now();
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    {
        std::lock_guard<std::mutex> lock(driver_mutex_);

        for (size_t idx = 0; idx < ids.size(); idx++)
        {            
            // ==== COMEÇO DA CONTAGEM INDIVIDUAL ====
            auto start = std::chrono::high_resolution_clock::now();
            // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
            
            uint16_t temp_pos;
            auto result = actuator_driver_->get_current_position(ids[idx], temp_pos);
            
            // ====== FIM DA CONTAGEM INDIVIDUAL ======
            auto end = std::chrono::high_resolution_clock::now(); // finaliza contagem de tempo
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
            
            // guarda o tempo individual (mesmo com falha)
            time_msg.names.push_back(names[idx]);
            time_msg.times.push_back(duration.count());

            // guarda infos de atuadores que enviaram posição
            if (result == 0) {
                successful_names.push_back(names[idx]);
                successful_positions.push_back(static_cast<int16_t>(temp_pos));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha na leitura do atuador %s. Erro: %d", 
                    names[idx].c_str(), static_cast<int>(result));
            }
        }
    }

    // só publica se pelo menos um atuador
    // tiver enviado dados
    if (!successful_names.empty()) {
        msg.names = successful_names;
        msg.positions = successful_positions;
        msg.header.stamp = this->get_clock()->now();
        state_publisher_->publish(msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "Nenhum atuador foi lido com sucesso. Publicação cancelada.");
    }

    // ======== FIM DA CONTAGEM TOTAL ========
    auto end_total = std::chrono::high_resolution_clock::now(); // fim do loop total
    auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total);
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    // publica os tempos individuais + tempo total
    time_msg.total_time = duration_total.count();
    time_publisher_->publish(time_msg);
}

void ActuatorNode::set_goal_position(const ActuatorCommand::SharedPtr msg)
{
    if (msg->names.empty() || msg->goals.empty()) return;

    std::vector<uint8_t> ids;
    std::vector<std::string> valid_names;

    // busca e salva o id correspondente ao respectivo nome
    for (const auto& name : msg->names)
    {
        auto id = parameter_manager_->get_id_by_name(name);
        if (id != -1)
        {
            ids.push_back(static_cast<uint8_t>(id));
            valid_names.push_back(name);
        }
    }

    // se nenhum nome bater, retorna
    if (valid_names.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Nenhum atuador válido nos nomes recebidos.");
        return;
    }

    // evita segfault
    size_t n = std::min(ids.size(), msg->goals.size()); 

    std::lock_guard<std::mutex> lock(driver_mutex_);
    
    // define o goal position para cada atuador
    for (size_t idx = 0; idx < n; idx++)
    {
        auto result = actuator_driver_->set_goal_position(ids[idx], msg->goals[idx]);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha no envio de Goal Position para o atuador %s. Erro: %d", 
            valid_names[idx].c_str(), static_cast<int>(result));
        }
    }
}

ActuatorNode::~ActuatorNode() {
    if (timing_log_.is_open()) {
        timing_log_.close();
    }
};
