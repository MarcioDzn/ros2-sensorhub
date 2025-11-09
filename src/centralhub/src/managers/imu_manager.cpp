#include "managers/imu_manager.hpp"

IMUManager::IMUManager(rclcpp::Node* node) : node_(node) {}

void IMUManager::loadParameters()
{
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.imu_ids", {1, 2, 3});
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.imu_addresses", {1, 0, 1});
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.euler_orders", {1, 2, 0, 1, 2, 0, 1, 2, 0});
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.multiplex_ids", {0, 1, 0});
    node_->declare_parameter<std::vector<std::string>>(
        "imu_manager.imu_names", {"sensor_1", "sensor_2", "sensor_3"});

    setParameters();
}


void IMUManager::createSensors()
{
    // criando e armazenando as instâncias
    // dos IMUs
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto imu = std::make_shared<BNO055IMU>(multiplex_ids_[id], imu_ids_[id], imu_addresses_[id]);
        imus_.push_back(imu);
    }
}


void IMUManager::createPublishers()
{
    // criando um publisher pra cada IMU
    auto qos = rclcpp::QoS(10).reliable();
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto publisher = node_->create_publisher<IMUData>(
            "/sensor" + std::to_string(imu_ids_[id]) + "/imu", qos);
        publishers_.push_back(publisher);
    }
}


void IMUManager::initialize()
{
    // setup dos IMUs
    for (auto& imu : imus_) imu->setup();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    // calibração dos IMUs
    for (auto& imu : imus_) imu->calibrate();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1s;
}


void IMUManager::publishAll()
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
            node_->get_logger(), 
            "[ID %zu] PUBLICANDO\nROLL: %f\nPITCH: %f\nYAW: %f", 
            id, message.roll, message.pitch, message.yaw);

        publishers_[id]->publish(message);
    }
}


void IMUManager::setParameters()
{
    imu_ids_ = node_->get_parameter("imu_manager.imu_ids").as_integer_array();
    imu_addresses_ = node_->get_parameter("imu_manager.imu_addresses").as_integer_array();
    multiplex_ids_ = node_->get_parameter("imu_manager.multiplex_ids").as_integer_array();
    imu_names_ = node_->get_parameter("imu_manager.imu_names").as_string_array();

    auto flat = node_->get_parameter("imu_manager.euler_orders").as_integer_array();
    euler_orders_ = chunkVector(flat, imu_ids_.size());
}


std::vector<std::vector<int>> IMUManager::chunkVector(
    const std::vector<int64_t>& flat, 
    size_t group_size
) 
{
    std::vector<std::vector<int>> chunks;

    if (flat.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Vetor de entrada 'flat' está vazio.");
        return chunks;
    }

    if (group_size == 0) {
        RCLCPP_ERROR(node_->get_logger(), "group_size não pode ser 0.");
        return chunks;
    }

    if (flat.size() % group_size != 0) {
        RCLCPP_WARN(node_->get_logger(),
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
