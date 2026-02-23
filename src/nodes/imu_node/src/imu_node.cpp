#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode(const rclcpp::NodeOptions & options) : Node("imu_node", options)
{
    timing_log_.open("tempos_imus.txt", std::ios::out | std::ios::trunc);

    parameter_manager_ = std::make_shared<ParameterManager>(this);
    manager_ = std::make_unique<IMUManager>(parameter_manager_);

    if (manager_->init(this) < 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware");
        throw std::runtime_error("");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    publisher_ = this->create_publisher<IMUState>(
        "imu/state", qos);

    // envia dados de tempo
    time_publisher_ = this->create_publisher<Time>(
        "imu/time", qos);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        std::bind(&IMUNode::state_callback, this));
}

void IMUNode::read_imu_data(Time& time_data)
{ 
    IMUState state_data;

    const auto& imus = manager_->get_imus();
    const auto& ids = parameter_manager_->get_ids();
    
    for (const auto& [id, imu] : imus)
    {
        std::vector<float> imu_quaternions_data;

        auto duration = measure_micros([&]() {
            imu->get_quaternions_data(imu_quaternions_data);
        });

        // TODO: adicionar nomes
        time_data.times.push_back(duration);

        // monta msg com dados do imu
        IMUData imu_data;
        imu_data.name = parameter_manager_->get_name(id);

        imu_data.q_w = imu_quaternions_data[0];
        imu_data.q_x = imu_quaternions_data[1];
        imu_data.q_y = imu_quaternions_data[2];
        imu_data.q_z = imu_quaternions_data[3];
        
        state_data.imus.push_back(imu_data);
    }

    return state_data;
}

void IMUNode::publish_imu_state() {
    Time time_msg;
    IMUState state_msg;

    auto duration = measure_micros([&]() {
        state_msg = read_imu_data(time_msg);
    });
    
    // TODO: adicionar nomes
    if (state_msg.imus.empty()) return;

    state_msg.header.stamp = this->get_clock()->now();
    state_publisher_->publish(state_msg);

    time_msg.total_time = duration;
    time_publisher_->publish(time_msg);
}

void IMUNode::state_callback()
{
    Time time_msg;  
    auto msg = IMUState();

    const auto& imus = manager_->get_imus();
    const auto& ids = parameter_manager_->get_ids();
    
    // ======= COMEÇO DA CONTAGEM TOTAL =======
    auto start_total = std::chrono::high_resolution_clock::now();
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    for (const auto& [id, imu] : imus)
    {
        std::vector<float> imu_quaternions_data;
        
        // ==== COMEÇO DA CONTAGEM INDIVIDUAL ====
        auto start = std::chrono::high_resolution_clock::now();
        // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

        imu->get_quaternions_data(imu_quaternions_data);

        // ====== FIM DA CONTAGEM INDIVIDUAL ======
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        // guarda o tempo individual (mesmo com falha)
        time_msg.names.push_back(parameter_manager_->get_name(id));
        time_msg.times.push_back(duration.count());
        
        IMUData data;
        data.name = parameter_manager_->get_name(id);

        data.q_w = imu_quaternions_data[0];
        data.q_x = imu_quaternions_data[1];
        data.q_y = imu_quaternions_data[2];
        data.q_z = imu_quaternions_data[3];
        
        msg.imus.push_back(data);
    }

    // publica os dados de todos os imus
    msg.header.stamp = this->get_clock()->now();
    publisher_->publish(msg);

    // ======== FIM DA CONTAGEM TOTAL ========
    auto end_total = std::chrono::high_resolution_clock::now(); 
    auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total);
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    // publica os tempos individuais + tempo total
    time_msg.total_time = duration_total.count();
    time_publisher_->publish(time_msg);
}

IMUNode::~IMUNode() = default;
