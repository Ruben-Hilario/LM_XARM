#include "xarm_control/xarm_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<xarm_control::OwnControl>(node_options);
    node->start();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}