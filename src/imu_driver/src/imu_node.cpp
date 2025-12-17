#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode() : Node("imu_node", rclcpp::NodeOptions()
           .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
{
    // inicializa o wiringpi
    BNO055IMU::setup_wiringpi();
    
    // carrega parametros
    load_parameters();
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    for (const auto &imu_config: imus_config_)
    {
        // cria e armazena as instâncias dos IMUs
        auto imu = std::make_shared<BNO055IMU>(imu_config.multiplexer, imu_config.id, imu_config.address);
        imus_.push_back(imu);
        
        // cria um publisher pra cada IMU
        RCLCPP_INFO(this->get_logger(), 
            "Criando publisher para o sensor %d", imu_config.id);
        auto publisher = this->create_publisher<IMUData>(
            "" + imu_config.name + "/imu", qos);
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
}

// callback que envia os dados dos 3 sensores
void IMUNode::timer_callback()
{
    for (size_t id = 0; id < imus_.size(); id++)
    {
        auto message = IMUData();
        std::vector<double> imu_data;
        
        imus_[id]->get_data(imu_data);
        message.roll = imu_data[0];
        message.pitch = imu_data[1];
        message.yaw = imu_data[2];
        
        message.stamp = this->get_clock()->now();


        publishers_[id]->publish(message);
    }
}

void IMUNode::parse_imus(std::vector<std::string> param_names)
{
    std::set<std::string> imu_names;
    // pega o nome do imu.
    // ex: sensor_1
    for (const auto &p : param_names) {
        auto rest = p.substr(5);
        auto name = rest.substr(0, rest.find('.'));
        imu_names.insert(name);
    }

    for (const auto &name : imu_names) {
        ImuConfig imu;
        imu.name = name;

        // ex: imu.id = imus.sensor_1.id
        if (!this->get_parameter("imus." + name + ".id", imu.id)) {
            throw std::runtime_error("IMU " + name + " sem id");
        }

        if (!this->get_parameter("imus." + name + ".address", imu.address)) {
            throw std::runtime_error("IMU " + name + " sem address");
        }

        if (this->has_parameter("imus." + name + ".multiplexer")) {
            this->get_parameter("imus." + name + ".multiplexer", imu.multiplexer);
        } else {
            imu.multiplexer = -1;
        }

        std::vector<int64_t> euler;
        if (!this->get_parameter("imus." + name + ".euler_order", euler)) {
            throw std::runtime_error("IMU " + name + " sem euler_order");
        }

        imu.euler_order[0] = static_cast<int>(euler[0]);
        imu.euler_order[1] = static_cast<int>(euler[1]);
        imu.euler_order[2] = static_cast<int>(euler[2]);

        imus_config_.push_back(imu);
    }
}

void IMUNode::load_parameters()
{
    set_parameters();
}

void IMUNode::set_parameters()
{
    auto param_names = this->list_parameters({"imus"}, 3).names;
    this->get_parameter("update_rate_ms", update_rate_ms_);
    parse_imus(param_names);
}

IMUNode::~IMUNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
