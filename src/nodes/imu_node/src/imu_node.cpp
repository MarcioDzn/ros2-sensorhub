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
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        std::bind(&IMUNode::state_callback, this));
}

void IMUNode::state_callback()
{
    std::vector<long> imu_times;
    static int loop_idx = 0;

    const auto& imus = manager_->get_imus();
    auto msg = IMUState();

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
        imu_times.push_back(duration.count());
        // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        
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

    // ======== GRAVA TODOS OS TEMPOS ========
    if (loop_idx >= 5) return;
    
    timing_log_ << (loop_idx + 1) << "\t";
    for (auto t_us : imu_times)
        timing_log_ << t_us << "\t";
    timing_log_ << duration_total.count() << "\n";
    
    loop_idx++;
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
}

IMUNode::~IMUNode() = default;
