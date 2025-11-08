#include "central_node.hpp"

using namespace std::chrono_literals;

CentralNode::CentralNode() : Node("central_node"), count_(0)
{
    // definindo parâmetros de inicialização
    declare_parameters();

    // leitura dos parâmetros
    load_parameters();

    // criando um publisher pra cada IMU
    auto qos = rclcpp::QoS(10).reliable();
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto publisher = this->create_publisher<IMUData>(
            "/sensor" + std::to_string(imu_ids_[id]) + "/imu", qos);
        publishers_.push_back(publisher);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_), 
        std::bind(&CentralNode::timer_callback, this));
}

// callback que recebe os dados dos 3 sensores
void CentralNode::timer_callback()
{
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto message = IMUData();
        std::vector<double> imu_data;

        get_imu_data(id, imu_data);

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

// mock de dados retornados por um IMU
void CentralNode::get_imu_data(int id, std::vector<double>& imu_data)
{
    imu_data.resize(3);
    imu_data[0] = id*15.43;
    imu_data[1] = id*28.12;
    imu_data[2] = id*1.54;
}

// converte um array plano em um array de arrays
// ex: {1, 2, 0, 1, 2, 0, 1, 2, 0}
// vira: {{1, 2, 0}, {1, 2, 0}, {1, 2, 0}}
std::vector<std::vector<int>> CentralNode::chunk_vector(
    const std::vector<int64_t>& flat, 
    size_t group_size
) {
    std::vector<std::vector<int>> chunks;

    if (flat.empty()) {
        RCLCPP_WARN(this->get_logger(), "Vetor de entrada 'flat' está vazio.");
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

void CentralNode::declare_parameters()
{
    this->declare_parameter<std::vector<int>>("imu_ids", {1, 2, 3});
    this->declare_parameter<std::vector<int>>("imu_addresses", {1, 0, 1});
    this->declare_parameter<std::vector<int>>("euler_orders", {1, 2, 0, 1, 2, 0, 1, 2, 0});
    this->declare_parameter<std::vector<int>>("multiplex_ids", {0, 1, 0});
    this->declare_parameter<std::vector<std::string>>(
        "imu_names", {"sensor_1", "sensor_2", "sensor_3"});
    this->declare_parameter<int>("update_rate_ms", 15);
}

void CentralNode::load_parameters()
{
    imu_ids_ = this->get_parameter("imu_ids").as_integer_array();
    imu_addresses_ = this->get_parameter("imu_addresses").as_integer_array();
    multiplex_ids_ = this->get_parameter("multiplex_ids").as_integer_array();
    imu_names_ = this->get_parameter("imu_names").as_string_array();
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
    auto flat = this->get_parameter("euler_orders").as_integer_array();
    euler_orders_ = chunk_vector(flat, imu_ids_.size());
}

CentralNode::~CentralNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}