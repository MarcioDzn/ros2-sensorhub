#include "sync_node.hpp"


SyncNode::SyncNode() : Node("sync_node") {}

SyncNode::~SyncNode() = default;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
