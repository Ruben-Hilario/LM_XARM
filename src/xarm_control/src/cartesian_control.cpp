#include "xarm_control/xarm_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto cartesian_node = std::make_shared<xarm_control::CartesianControl>(node_options);
    cartesian_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    cartesian_node->execute_cartesian_plan();

    rclcpp::shutdown();
    return 0;
}