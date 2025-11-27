#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode() : Node("imu_node")
{
    // inicializa o wiringpi
    BNO055IMU::setup_wiringpi();
    
    // carrega parametros
    load_parameters();
    
    auto qos = rclcpp::QoS(10).reliable();
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        // cria e armazena as instâncias dos IMUs
        auto imu = std::make_shared<BNO055IMU>(multiplex_ids_[id], imu_ids_[id], imu_addresses_[id]);
        imus_.push_back(imu);
        
        // cria um publisher pra cada IMU
        RCLCPP_INFO(this->get_logger(), 
            "Criando publisher para o sensor %d", imu_ids_[id]);
        auto publisher = this->create_publisher<IMUData>(
            "" + imu_names_[id] + "/imu", qos);
        publishers_.push_back(publisher);
    }
    
    // setup dos IMUs
    for (auto& imu : imus_) imu->setup();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    // calibra os IMUs
    for (auto& imu : imus_) imu->calibrate();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1s;
    
    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_), 
        std::bind(&IMUNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "DEBUG 2");
}

// callback que envia os dados dos 3 sensores
void IMUNode::timer_callback()
{
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto message = IMUData();
        std::vector<double> imu_data;
        
        imus_[id]->get_data(imu_data);
        message.roll = imu_data[0];
        message.pitch = imu_data[1];
        message.yaw = imu_data[2];

        RCLCPP_INFO(
            this->get_logger(), 
            "[ID %zu] PUBLICANDO\nROLL: %f\nPITCH: %f\nYAW: %f", 
            id, message.roll, message.pitch, message.yaw);
        publishers_[id]->publish(message);
    }
}

std::vector<std::vector<int>> IMUNode::chunk_vector(
    const std::vector<int64_t>& flat, 
    size_t group_size
) 
{
    std::vector<std::vector<int>> chunks;

    if (flat.empty()) {
        RCLCPP_WARN(this->get_logger(), "Vetor de entrada está vazio.");
        return chunks;
    }

    if (group_size == 0) {
        RCLCPP_ERROR(this->get_logger(), "group_size não pode ser 0.");
        return chunks;
    }

    if (flat.size() % group_size != 0) {
        RCLCPP_WARN(this->get_logger(),
            "Tamanho do vetor (%zu) não é múltiplo de group_size (%zu). "
            "O último grupo pode ficar incompleto.",
            flat.size(), group_size);
    }

    // divide o vetor plano em blocos de tamanho group_size
    for (size_t i = 0; i < flat.size(); i += group_size) {
        size_t end = std::min(i + group_size, flat.size());
        chunks.emplace_back(flat.begin() + i, flat.begin() + end);
    }

    return chunks;
}

void IMUNode::load_parameters()
{
    this->declare_parameter<int>("update_rate_ms", 15);
    this->declare_parameter<std::vector<int64_t>>("imu_ids", {1, 2, 3});
    this->declare_parameter<std::vector<int64_t>>("imu_addresses", {0x28, 0x29, 0x28});
    this->declare_parameter<std::vector<int64_t>>("euler_orders", {1, 2, 0, 1, 2, 0, 1, 2, 0});
    this->declare_parameter<std::vector<int64_t>>("multiplex_ids", {0, 1, 0});
    this->declare_parameter<std::vector<std::string>>(
        "imu_names", {"sensor_1", "sensor_2", "sensor_3"});

    set_parameters();
}

void IMUNode::set_parameters()
{
    imu_ids_ = this->get_parameter("imu_ids").as_integer_array();
    imu_addresses_ = this->get_parameter("imu_addresses").as_integer_array();
    multiplex_ids_ = this->get_parameter("multiplex_ids").as_integer_array();
    imu_names_ = this->get_parameter("imu_names").as_string_array();
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();

    auto flat = this->get_parameter("euler_orders").as_integer_array();
    euler_orders_ = chunk_vector(flat, imu_ids_.size());
}

IMUNode::~IMUNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
