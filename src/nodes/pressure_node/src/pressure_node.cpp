#include <vector>
#include <sstream>

#include "pressure_node.hpp"
#include "driver/pressure_factory.hpp"

using namespace std::chrono_literals;

PressureNode::PressureNode(const rclcpp::NodeOptions& options) 
    : Node("pressure_node", options)
{
    timing_log_.open("tempos_pressure.txt", std::ios::out | std::ios::trunc);

    parameter_manager_ = std::make_shared<ParameterManager>(this);
    pressure_drivers_.resize(parameter_manager_->get_ids().size());

    // inicializa cada sensor
    for (size_t idx = 0; idx < parameter_manager_->get_ids().size(); idx++)
    {
        pressure_drivers_[idx] = PressureFactory::create_pressure();

        if (pressure_drivers_[idx]->init(
            parameter_manager_->get_usb_ports()[idx], 
            parameter_manager_->get_baudrate()) < 0) {
                RCLCPP_FATAL(this->get_logger(), 
                    "Falha na inicialização do hardware serial na porta %s.", 
                    parameter_manager_->get_usb_ports()[idx].c_str());
                throw std::runtime_error("Falha ao inicializar PressureNode"); 
        }
    }
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    publisher_ = this->create_publisher<PressureState>(
        "pressure/state", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        std::bind(&PressureNode::publish_pressure_data, this));
}

void PressureNode::publish_pressure_data()
{
    std::vector<long> pressure_times;
    static int loop_idx = 0;

    auto msg = PressureState();
    auto ids = parameter_manager_->get_ids();
    auto names = parameter_manager_->get_names();
    
    // evita segfault
    size_t min_size = std::min(ids.size(), names.size());
    msg.names.reserve(min_size);
    msg.pressures.reserve(min_size);

    // ====== COMEÇO DA CONTAGEM TOTAL =======
    auto start_total = std::chrono::high_resolution_clock::now();
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    uint8_t error_count = 0;
    for (size_t idx = 0; idx < min_size; idx++)
    {   
        // ==== COMEÇO DA CONTAGEM INDIVIDUAL ====
        auto start = std::chrono::high_resolution_clock::now(); 
        // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

        // lê os dados da STM
        std::vector<uint16_t> data;
        if (pressure_drivers_[idx]->get_data(data) != 0)
        {
            error_count++; continue;
        }
        
        // ====== FIM DA CONTAGEM INDIVIDUAL ======
        auto end = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        pressure_times.push_back(duration.count());
        // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        PressureData pd;
        pd.pressures.reserve(data.size()); // prepara o vetor de cada ponto de pressão

        size_t sensor_id = 0;
        for (auto val : data)
        {
            PressureUnitSensor unit_sensor;
            unit_sensor.id = sensor_id++;
            unit_sensor.pressure = static_cast<int16_t>(val);
            pd.pressures.push_back(unit_sensor);
        }

        msg.pressures.push_back(pd);
        msg.names.push_back(parameter_manager_->get_names()[idx]);
        msg.header.stamp = this->get_clock()->now();
    }

    // se nenhuma palmilha enviou a posição
    // entao nao publica nada
    if (error_count < min_size) 
    {
        publisher_->publish(msg);
    }   

    // ======== FIM DA CONTAGEM TOTAL ========
    auto end_total = std::chrono::high_resolution_clock::now(); 
    auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total);
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    // ========= GRAVA TODOS OS TEMPO =========
    if (loop_idx >= 5) return;
    
    timing_log_ << (loop_idx + 1) << "\t";
    for (auto t_us : pressure_times)
        timing_log_ << t_us << "\t";
    timing_log_ << duration_total.count() << "\n";
    
    loop_idx++;
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

}

PressureNode::~PressureNode() = default;
