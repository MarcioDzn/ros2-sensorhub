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
        parameter_manager_->get_base_name() + "/state", qos);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        std::bind(&IMUNode::state_callback, this));
}

void IMUNode::state_callback()
{
    const auto& imus = manager_->get_imus();
    auto msg = IMUState();

    const auto& ids = parameter_manager_->get_ids();

    static std::map<int, int> loop_count_map;

    static int total_loop_count = 0;                  // contador do loop total
    auto start_total = std::chrono::high_resolution_clock::now();

    for (const auto& [id, imu] : imus)
    {
        std::vector<float> imu_euler_data;
        std::vector<float> imu_quaternions_data;
        
        auto start = std::chrono::high_resolution_clock::now();

        imu->get_euler_data(imu_euler_data);
        imu->get_quaternions_data(imu_quaternions_data);

        auto end = std::chrono::high_resolution_clock::now(); // finaliza contagem de tempo

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // registra no log se ainda não atingiu 5 loops para este motor
        if (timing_log_.is_open() && loop_count_map[ids[id]] < 5) {
            loop_count_map[ids[id]]++; // incrementa contador do motor
            timing_log_ << ids[id] 
                        << " loop" << loop_count_map[ids[id]] 
                        << " tempo_us=" << duration.count() << "\n";
        }

        IMUData data;
        data.id = id;

        // euler angles
        data.roll = imu_euler_data[0];
        data.pitch = imu_euler_data[1];
        data.yaw = imu_euler_data[2];

        // quaternions
        data.q_w = imu_quaternions_data[0];
        data.q_x = imu_quaternions_data[1];
        data.q_y = imu_quaternions_data[2];
        data.q_z = imu_quaternions_data[3];
        
        msg.imus.push_back(data);
    }

    msg.header.stamp = this->get_clock()->now();
    publisher_->publish(msg);

    // FIM CONTAGEM DE TEMPO
    auto end_total = std::chrono::high_resolution_clock::now(); // fim do loop total
    auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total);

    // registra o tempo total apenas 5 vezes
    if (timing_log_.is_open() && total_loop_count < 5) {
        total_loop_count++;
        timing_log_ << "LOOP_TOTAL loop" << total_loop_count
                    << " tempo_us=" << duration_total.count() << "\n";
    }
}

IMUNode::~IMUNode() = default;
