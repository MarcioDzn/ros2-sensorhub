#include "node/actuator_node.hpp"

#include "control/node/parameter_manager.hpp"

#include "driver/actuator_factory.hpp"

#include <vector>

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions& options) 
    : Node("actuator_node", options)
{   
    // log
    timing_log_.open("tempos_atuadores.txt", std::ios::out | std::ios::trunc);

    parameter_manager_ = std::make_shared<ParameterManager>(this);
    actuator_driver_ = ActuatorFactory::createDynamixel();

    // inicializa porta serial inserida nos parâmetros do yaml
    if (actuator_driver_->init(
        parameter_manager_->get_usb_port(), 
        parameter_manager_->get_baudrate()) != 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial na porta %s.", 
            parameter_manager_->get_usb_port().c_str());
        throw std::runtime_error("Falha ao inicializar ActuatorNode");
    }

    // TODO: verificar se os ids fornecidos pelo yaml
    // são de atuadores realmente conectados

    // envio de posições dos atuadores DEVEM chegar (?)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    // recebe dados de comando contínuo
    // ex: posição alvo
    actuator_subscriber_ = this->create_subscription<Command>(
        parameter_manager_->get_base_name() + "/command", qos, 
        [this](const Command::SharedPtr msg) {
            
            this->goal_position_callback(msg);
        });
    
    // recebe dados de comando pontual
    // ex: habilitar/desabilitar torque
    set_torque_service_ = this->create_service<SetTorque>(
        parameter_manager_->get_base_name() + "/set_torque", 
        [this](const std::shared_ptr<SetTorque::Request> req, 
                std::shared_ptr<SetTorque::Response> res) {
            this->set_torque_service_callback(req, res);
        });

    // envia dados de estado
    // ex: posição atual
    state_publisher_ = this->create_publisher<State>(
        parameter_manager_->get_base_name() + "/state", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        [this]() {
            state_callback();
        });

    RCLCPP_INFO(this->get_logger(), "Nó ActuatorNode iniciado com sucesso.");
}

void ActuatorNode::set_torque_service_callback(
    const std::shared_ptr<SetTorque::Request> request,
    std::shared_ptr<SetTorque::Response> response)
{
    std::lock_guard<std::mutex> lock(driver_mutex_);

    auto id = parameter_manager_->get_id_by_name(request->name);

    if (id == -1){
        RCLCPP_WARN(this->get_logger(), "Atuador %s não encontrado", 
            request->name.c_str());
        response->success = false;
        return;
    }

    auto result = actuator_driver_->set_torque(static_cast<uint8_t>(id), request->status ? 1 : 0);

    if (result != 0) {
        RCLCPP_WARN(this->get_logger(), 
            "Falha na configuração do torque. Erro: %d", 
            static_cast<int>(result));
        response->success = false;
        return; 
    }
    
    response->success = true;
}

void ActuatorNode::state_callback()
{
    State msg;

    const auto& ids = parameter_manager_->get_ids();
    const auto& names = parameter_manager_->get_names();

    std::vector<long> actuator_times;
    static int loop_idx = 0;

    std::vector<std::string> successful_names;
    std::vector<int16_t> successful_positions;

    // COMEÇO DA CONTAGEM TOTAL
    auto start_total = std::chrono::high_resolution_clock::now();

    {
        std::lock_guard<std::mutex> lock(driver_mutex_);

        for (size_t idx = 0; idx < ids.size(); idx++)
        {
            uint16_t temp_pos;
            
            // COMEÇO DA CONTAGEM INDIVIDUAL
            auto start = std::chrono::high_resolution_clock::now(); // inicia contagem de tempo
            
            auto result = actuator_driver_->get_current_position(ids[idx], temp_pos);
            
            // FIM DA CONTAGEM INDIVIDUAL
            auto end = std::chrono::high_resolution_clock::now(); // finaliza contagem de tempo
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

            actuator_times.push_back(duration.count());

            if (result == 0) {
                // adiciona apenas os nomes e posições válidos
                successful_names.push_back(names[idx]);
                successful_positions.push_back(static_cast<int16_t>(temp_pos));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha na leitura do atuador %s. Erro: %d", 
                    names[idx].c_str(), static_cast<int>(result));
            }
        }
    }

    // só publica se houver pelo menos um item válido
    if (!successful_names.empty()) {
        msg.names = successful_names;
        msg.positions = successful_positions;
        msg.header.stamp = this->get_clock()->now();
        state_publisher_->publish(msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "Nenhum atuador foi lido com sucesso. Publicação cancelada.");
    }

    // FIM DA CONTAGEM TOTAL
    auto end_total = std::chrono::high_resolution_clock::now(); // fim do loop total
    auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total);

    // grava todos os tempos
    if (loop_idx >= 5) return;
    
    timing_log_ << (loop_idx + 1) << "\t";
    for (auto t_us : actuator_times)
        timing_log_ << t_us << "\t";
    timing_log_ << duration_total.count() << "\n";
    
    loop_idx++;
}

void ActuatorNode::goal_position_callback(const Command::SharedPtr msg)
{
    if (msg->names.empty() || msg->goals.empty()) return;

    std::vector<uint8_t> ids;
    std::vector<std::string> valid_names;

    // busca o id correspondente ao nome
    for (const auto& name : msg->names)
    {
        auto id = parameter_manager_->get_id_by_name(name);
        if (id != -1)
        {
            ids.push_back(static_cast<uint8_t>(id));
            valid_names.push_back(name);
        }
    }

    if (valid_names.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Nenhum atuador válido nos nomes recebidos.");
        return;
    }

    size_t n = std::min(ids.size(), msg->goals.size()); // menor tamanho

    std::lock_guard<std::mutex> lock(driver_mutex_);
    
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
