#ifndef XARM_CONTROL_HPP_
#define XARM_CONTROL_HPP_

#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <std_srvs/srv/trigger.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>

namespace xarm_control
{

// ─── Gripper ───────────────────────────────────────────────────────────────
class GripperController : public rclcpp::Node
{
public:
    GripperController();
    void open_gripper();
    void close_gripper();

private:
    void publish_gripper_trajectory(double gripper_angle);
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher_;
    double gripper_open_angle_;
    double gripper_closed_angle_;
};

// ─── Cartesian Planning (MoveGroup) ────────────────────────────────────────
class CartesianControl : public rclcpp::Node
{
public:
    CartesianControl(const rclcpp::NodeOptions & options);
    ~CartesianControl() = default;
    void execute_cartesian_plan();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    geometry_msgs::msg::Pose target_pose_;
    bool pose_received_;
    rclcpp::Logger logger_;
};

// ─── MoveIt Servo keyboard node ────────────────────────────────────────────
class MoveitServo : public rclcpp::Node
{
public:
    explicit MoveitServo(const rclcpp::NodeOptions & options);
    ~MoveitServo();

    /// Blocking call: reads keyboard and streams servo commands until 'Q' pressed.
    void run();

private:
    // -- keyboard helpers --
    void setup_terminal();
    void restore_terminal();
    /// Reads one raw character (blocking). Returns false if shutdown requested.
    bool read_char(char & c);

    // -- servo helpers --
    void switch_command_type(int type);   // 0=JOINT_JOG, 1=TWIST
    void publish_twist(double lx, double ly, double lz,
                       double ax = 0, double ay = 0, double az = 0);
    void publish_joint_jog(const std::string & joint_name, double velocity);

    // -- ROS interfaces --
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr        joint_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr   switch_client_;

    // -- config --
    std::string planning_frame_;
    std::string base_frame_;
    std::string ee_frame_;
    double linear_speed_;
    double angular_speed_;
    double joint_speed_;
    int    joint_dir_;        ///< +1 or -1, toggled by 'R'
    int    command_type_;     ///< currently active servo command type

    // -- terminal state --
    struct termios old_termios_;
    bool terminal_set_;

    // -- threading --
    std::atomic<bool> running_;
    std::thread spin_thread_;

    rclcpp::Logger logger_;
};

// ─── Autonomous Control (OwnControl) ───────────────────────────────────────
class OwnControl : public rclcpp::Node
{
public:
    explicit OwnControl(const rclcpp::NodeOptions & options);
    ~OwnControl() = default;

    void start();

private:
    void timer_callback();
    void switch_command_type(int type);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string ee_frame_;
    rclcpp::Time start_time_;
};

} // namespace xarm_control
#endif  // XARM_CONTROL_HPP_