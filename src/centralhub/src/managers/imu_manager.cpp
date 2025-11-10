#include "managers/imu_manager.hpp"

/**
 * @brief Construtor da classe IMUManager.
 * 
 * @param node Ponteiro para o nó ROS2 principal que gerencia os parâmetros e publishers.
 * 
 * A classe usa esse ponteiro para acessar e declarar parâmetros, criar publishers e logar mensagens.
 */
IMUManager::IMUManager(rclcpp::Node* node) : node_(node) {}

/**
 * @brief Declara e carrega os parâmetros relacionados às IMUs.
 * 
 * Este método define valores padrão no ROS2 (caso não estejam no arquivo YAML)
 * e logo em seguida chama `setParameters()` para armazená-los em variáveis da classe.
 */
void IMUManager::loadParameters()
{
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.imu_ids", {1, 2, 3});
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.imu_addresses", {0x28, 0x29, 0x28});
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.euler_orders", {1, 2, 0, 1, 2, 0, 1, 2, 0});
    node_->declare_parameter<std::vector<int64_t>>("imu_manager.multiplex_ids", {0, 1, 0});
    node_->declare_parameter<std::vector<std::string>>(
        "imu_manager.imu_names", {"sensor_1", "sensor_2", "sensor_3"});

    setParameters();
}

/**
 * @brief Cria e inicializa os objetos BNO055IMU com base nos parâmetros carregados.
 * 
 * Cada IMU é representada por um objeto `BNO055IMU` associado ao seu ID, endereço e multiplexador.
 */
void IMUManager::createSensors()
{
    // criando e armazenando as instâncias
    // dos IMUs
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        RCLCPP_INFO(node_->get_logger(), 
            "Criando sensor %d - MULTIPLEX ID: %d IMU ID: %d ADDRESS: %d",
            multiplex_ids_[id], imu_ids_[id], imu_addresses_[id]);
        auto imu = std::make_shared<BNO055IMU>(multiplex_ids_[id], imu_ids_[id], imu_addresses_[id]);
        imus_.push_back(imu);
    }
}

/**
 * @brief Cria um publisher ROS2 para cada IMU configurada.
 * 
 * Cada publisher publica mensagens do tipo `IMUData` em um tópico distinto:
 * 
 * Exemplo:  
 * `/sensor1/imu`, `/sensor2/imu`, `/sensor3/imu`
 */
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

/**
 * @brief Inicializa e calibra todas as IMUs.
 * 
 * - Chama o método `setup()` de cada IMU para inicialização.  
 * 
 * - Aguarda 1 ms entre as configurações.  
 * 
 * - Depois chama `calibrate()` em todas as IMUs, com espera de 1 segundo.
 */
void IMUManager::initialize()
{
    // setup do wiring pi
    BNO055IMU::setup_wiringpi();
    
    // setup dos IMUs
    for (auto& imu : imus_) imu->setup();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    // calibração dos IMUs
    for (auto& imu : imus_) imu->calibrate();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1s;
}

/**
 * @brief Publica os dados de todas as IMUs.
 * 
 * Lê os valores de cada IMU (roll, pitch, yaw), preenche a mensagem `IMUData`
 * e publica no tópico correspondente.
 */

 /**
 * @brief Publica os dados de todas as IMUs.
 * 
 * Lê os valores de cada IMU (roll, pitch, yaw), preenche a mensagem `IMUData`
 * e publica no tópico correspondente.
 * 
 *
 */
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


/**
 * @brief Lê os parâmetros previamente declarados e armazena-os em variáveis da classe.
 * 
 * Os parâmetros são lidos diretamente do nó ROS2 e usados em métodos como `createSensors()`.
 * 
 * Também realiza o processamento do vetor achatado `euler_orders` em subvetores com `chunkVector()`.
 * 
 * 
 */
void IMUManager::setParameters()
{
    imu_ids_ = node_->get_parameter("imu_manager.imu_ids").as_integer_array();
    imu_addresses_ = node_->get_parameter("imu_manager.imu_addresses").as_integer_array();
    multiplex_ids_ = node_->get_parameter("imu_manager.multiplex_ids").as_integer_array();
    imu_names_ = node_->get_parameter("imu_manager.imu_names").as_string_array();

    auto flat = node_->get_parameter("imu_manager.euler_orders").as_integer_array();
    euler_orders_ = chunkVector(flat, imu_ids_.size());
}

/**
 * @brief Divide um vetor plano em grupos de tamanho `group_size`.
 * 
 * @param flat Vetor de inteiros achatado (ex: {1,2,0, 1,2,0, 1,2,0})
 * @param group_size Número de elementos por grupo.
 * @return std::vector<std::vector<int>> Vetor de vetores com os grupos formados.
 * 
 * Caso o tamanho do vetor não seja múltiplo de `group_size`, 
 * o último grupo poderá ficar incompleto.
 * 
 * 
 */
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
