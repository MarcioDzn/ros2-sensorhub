#include "controller_node.hpp"

using namespace std::chrono_literals;

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options) 
    : Node("controller_node", options)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();


    actuator_publisher_ = this->create_publisher<interfaces::msg::MG8008ECommand>("/actuator/command", 10);
    actuator_subscriber_ = this->create_subscription<interfaces::msg::MG8008EState>(
            "/actuator/state", 
            10, 
            std::bind(&ControllerNode::get_angle, this, std::placeholders::_1
        )
    );

    publisher_ = this->create_publisher<interfaces::msg::ControllerOut>(
        "controller/state", qos);
    subscriber_ = this->create_subscription<interfaces::msg::ControllerIn>(
        "controller/command",
        qos,
        std::bind(
            &ControllerNode::controller_callback,
            this,
            std::placeholders::_1
        )
    );

    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&ControllerNode::publish_state, this)
    );
}

void ControllerNode::get_angle(const interfaces::msg::MG8008EState::SharedPtr msg)
{
    real_angle_ = msg->angles[0]; // apenas do primeiro atuador
}

void ControllerNode::send_angle(std::string name, int32_t angle, int32_t speed) {
    interfaces::msg::MG8008ECommand actuator_msg;
    actuator_msg.names.push_back(name);
    actuator_msg.angles.push_back(angle);
    actuator_msg.speeds.push_back(speed);
    actuator_publisher_->publish(actuator_msg);
}

void ControllerNode::publish_state()
{
    interfaces::msg::ControllerOut msg;

    msg.name = name_; 
    msg.ref_angle = ref_angle_.load();
    msg.real_angle = real_angle_.load();

    publisher_->publish(msg);
}

void ControllerNode::controller_callback(
    const interfaces::msg::ControllerIn::SharedPtr msg)
{
    ref_angle_ = msg->ref_angle;
    name_ = msg->name;

    int16_t speed = msg->speed;
    send_angle(name_, ref_angle_.load(), speed);

    // muda periodo de feedback
    if (msg->feedback_period_us > 0 &&
        msg->feedback_period_us != current_period_us_)
    {
        current_period_us_ = msg->feedback_period_us;

        feedback_period_ = std::chrono::microseconds(current_period_us_);

        timer_->cancel();

        timer_ = this->create_wall_timer(
            feedback_period_,
            std::bind(&ControllerNode::publish_state, this)
        );
    }
}

ControllerNode::~ControllerNode() = default;
