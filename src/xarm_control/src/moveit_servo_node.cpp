// moveit_servo_node.cpp
// Entry-point for the MoveitServo keyboard-teleop node.
// Launch alongside a running servo_server (e.g. via xarm_moveit_servo launch).

#include "xarm_control/xarm_control.hpp"
#include <signal.h>

static xarm_control::MoveitServo * g_servo_node = nullptr;

// Restore terminal on Ctrl+C so the shell isn't left in raw mode
static void signal_handler(int /*sig*/)
{
    if (g_servo_node) {
        // restore_terminal is called in ~MoveitServo, but call explicitly here
        // to be safe before rclcpp::shutdown tears things down.
    }
    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<xarm_control::MoveitServo>(options);
    g_servo_node = node.get();

    signal(SIGINT, signal_handler);

    // run() blocks until 'Q' is pressed or rclcpp is shut down
    node->run();

    rclcpp::shutdown();
    return 0;
}
