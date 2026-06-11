#include "xarm_control/xarm_control.hpp"
namespace xarm_control{

GripperController::GripperController() : Node("gripper_controller"){
    gripper_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/xarm_gripper_traj_controller/joint_trajectory", 10);
    gripper_open_angle_ = 49.0 * M_PI / 180.0;
    gripper_closed_angle_ = 0.0;
}

void GripperController::open_gripper()
{
    publish_gripper_trajectory(gripper_open_angle_);
}

void GripperController::close_gripper()
{
    publish_gripper_trajectory(gripper_closed_angle_);
}

void GripperController::publish_gripper_trajectory(double gripper_angle)
{
    auto gripper_trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    gripper_trajectory_msg.joint_names.push_back("drive_joint");

    auto gripper_point = trajectory_msgs::msg::JointTrajectoryPoint();
    gripper_point.positions.push_back(gripper_angle);
    gripper_point.velocities.push_back(0.05);
    gripper_point.accelerations.push_back(0.05);
    gripper_point.time_from_start.sec = 2;

    gripper_trajectory_msg.points.push_back(gripper_point);
    gripper_publisher_->publish(gripper_trajectory_msg);
    
    RCLCPP_INFO(this->get_logger(), "Publicando trayectoria del gripper: %s",
                gripper_angle == gripper_open_angle_ ? "Abierto" : "Cerrado");
}

CartesianControl::CartesianControl(const rclcpp::NodeOptions & options) 
: Node("cartesian_control_node", options), pose_received_(false), logger_(rclcpp::get_logger("move_group_demo"))
{
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ur5e_goal_pose", 10,
        std::bind(&CartesianControl::pose_callback, this, std::placeholders::_1));
}

void CartesianControl::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    target_pose_ = msg->pose;
    pose_received_ = true; 
}

void CartesianControl::execute_cartesian_plan()
{
    rclcpp::SyncParametersClient param_client(shared_from_this(), "move_group");
    if (!param_client.wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(logger_, "move_group service not available, check if MoveIt is running");
        return;
    }

    std::vector<std::string> param_names = {
        "robot_description",
        "robot_description_semantic",
        "robot_description_kinematics",
        "robot_description_planning"
    };

    auto params = param_client.get_parameters(param_names);
    std::vector<rclcpp::Parameter> new_params;
    for (size_t i = 0; i < param_names.size(); ++i) {
        if (params[i].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
            new_params.push_back(params[i]);
            RCLCPP_INFO(logger_, "Loaded parameter: %s", param_names[i].c_str());
        } else {
            RCLCPP_WARN(logger_, "Parameter %s not found in /move_group", param_names[i].c_str());
        }
    }
    for (const auto &param : new_params) {
        this->declare_parameter(param.get_name(), param.get_value<rclcpp::ParameterValue>());
    }
    this->set_parameters(new_params);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_from_this());
    std::thread([&executor]() { executor.spin(); }).detach();

    while (!pose_received_) {
        RCLCPP_INFO(logger_, "Waiting for pose...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    static const std::string PLANNING_GROUP_ARM = "xarm7";
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP_ARM);
    move_group.setEndEffectorLink("link_eef");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    RCLCPP_INFO(logger_, "Approaching target pose with Cartesian path...");
    RCLCPP_INFO(logger_, "Target Pose: x=%.3f, y=%.3f, z=%.3f",
                target_pose_.position.x, target_pose_.position.y, target_pose_.position.z);

    auto gripper_node = std::make_shared<GripperController>();
    gripper_node->close_gripper();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose mid_pose = target_pose_;
    mid_pose.position.z += 0.05;
    waypoints.push_back(mid_pose);
    waypoints.push_back(target_pose_);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory, true);

    if (fraction > 0.99) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;

        for (const auto& point : trajectory.joint_trajectory.points) {
            for (const auto& posit : point.positions) {
                std::cout << posit << ", ";
            }
            std::cout << std::endl;
        }

        std::cout << "**************************************** second vel  ****************" << std::endl;

        robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
        trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
        time_param.computeTimeStamps(rt, 0.02, 0.02);

        rt.getRobotTrajectoryMsg(trajectory);

        for (const auto& point : trajectory.joint_trajectory.points) {
            for (const auto& posit : point.positions) {
                std::cout << posit << ", ";
            }
            std::cout << std::endl;
        }

        plan.trajectory = trajectory;
        move_group.execute(plan);

        RCLCPP_INFO(logger_, "Movement executed successfully.");
        gripper_node->open_gripper();
    } else {
        RCLCPP_WARN(logger_, "Cartesian path planning failed, only achieved %.2f%% of the path", fraction * 100.0);
    }
}


}

// ─── MoveitServo Implementation ─────────────────────────────────────────────
// Key bindings:
//   Arrow UP/DOWN  → linear X  (forward/back in base frame)
//   Arrow LEFT/RIGHT → linear Y (left/right in base frame)
//   ';' / '.'      → linear Z  (up/down)
//   'W'            → use base frame for commands
//   'E'            → use end-effector frame for commands
//   '[' / ']'      → angular Z (yaw)
//   '1'-'7'        → joint jog for joint1..joint7
//   'R'            → reverse joint jog direction
//   'Q'            → quit
// ─────────────────────────────────────────────────────────────────────────────

namespace xarm_control {

// Arrow keys arrive as 3-byte escape sequences: ESC [ A/B/C/D
static constexpr char KEYCODE_ESC       = 0x1B;
static constexpr char KEYCODE_BRACKET   = 0x5B;  // '['
static constexpr char KEYCODE_UP        = 0x41;
static constexpr char KEYCODE_DOWN      = 0x42;
static constexpr char KEYCODE_RIGHT     = 0x43;
static constexpr char KEYCODE_LEFT      = 0x44;
// Regular keys
static constexpr char KEYCODE_PERIOD    = 0x2E;  // '.'  → -Z
static constexpr char KEYCODE_SEMICOLON = 0x3B;  // ';'  → +Z
static constexpr char KEYCODE_RBRACKET  = 0x5D;  // ']'  → +yaw
static constexpr char KEYCODE_E         = 0x65;
static constexpr char KEYCODE_W         = 0x77;
static constexpr char KEYCODE_Q         = 0x71;
static constexpr char KEYCODE_R         = 0x72;
static constexpr char KEYCODE_1         = 0x31;
static constexpr char KEYCODE_7         = 0x37;

MoveitServo::MoveitServo(const rclcpp::NodeOptions & options)
: Node("moveit_servo_keyboard_node", options),
  planning_frame_("link_base"),
  base_frame_("link_base"),
  ee_frame_("link_eef"),
  linear_speed_(0.10),    // m/s
  angular_speed_(0.3),    // rad/s
  joint_speed_(0.5),      // rad/s
  joint_dir_(1),
  command_type_(-1),
  terminal_set_(false),
  running_(false),
  logger_(rclcpp::get_logger("moveit_servo_keyboard"))
{
    // Declare / load parameters (can be overridden via yaml or launch args)
    auto declare_or_get_string = [this](const std::string& name, std::string& var) {
        if (!this->has_parameter(name)) { this->declare_parameter<std::string>(name, var); }
        this->get_parameter(name, var);
    };
    auto declare_or_get_double = [this](const std::string& name, double& var) {
        if (!this->has_parameter(name)) { this->declare_parameter<double>(name, var); }
        this->get_parameter(name, var);
    };

    declare_or_get_string("planning_frame", planning_frame_);
    declare_or_get_string("ee_frame", ee_frame_);
    declare_or_get_string("base_frame", base_frame_);
    declare_or_get_double("linear_speed", linear_speed_);
    declare_or_get_double("angular_speed", angular_speed_);
    declare_or_get_double("joint_speed", joint_speed_);

    // Publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_server/delta_twist_cmds", 10);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        "/servo_server/delta_joint_cmds", 10);

    // Service clients
    switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_server/switch_command_type");

    RCLCPP_INFO(logger_, "MoveitServo node constructed. Call run() to start keyboard control.");
}

MoveitServo::~MoveitServo()
{
    running_ = false;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    restore_terminal();
}

// ─── terminal helpers ────────────────────────────────────────────────────────
void MoveitServo::setup_terminal()
{
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios raw = old_termios_;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    terminal_set_ = true;
}

void MoveitServo::restore_terminal()
{
    if (terminal_set_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        terminal_set_ = false;
    }
}

bool MoveitServo::read_char(char & c)
{
    int rc = read(STDIN_FILENO, &c, 1);
    return rc == 1 && running_;
}

// ─── servo helpers ────────────────────────────────────────────────────────────

void MoveitServo::switch_command_type(int type)
{
    if (type == command_type_) return;
    // Non-blocking: fire and forget — we don't need to wait for confirmation
    // because the spin thread will handle the response callback.
    if (!switch_client_->service_is_ready()) return;

    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    switch (type) {
        case 0: req->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG; break;
        case 1: req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;     break;
        default: return;
    }
    // Optimistically assume success; if it fails the servo will reject the msg.
    command_type_ = type;
    switch_client_->async_send_request(req,
        [this, type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture f) {
            if (!f.get()->success) {
                RCLCPP_WARN(logger_, "switch_command_type(%d) rejected by servo_server.", type);
                command_type_ = -1;  // force retry next time
            }
        }
    );
}

void MoveitServo::publish_twist(double lx, double ly, double lz,
                                double ax, double ay, double az)
{
    switch_command_type(1);
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp    = this->now();
    msg->header.frame_id = planning_frame_;
    msg->twist.linear.x  = lx;
    msg->twist.linear.y  = ly;
    msg->twist.linear.z  = lz;
    msg->twist.angular.x = ax;
    msg->twist.angular.y = ay;
    msg->twist.angular.z = az;
    twist_pub_->publish(std::move(msg));
}

void MoveitServo::publish_joint_jog(const std::string & joint_name, double velocity)
{
    switch_command_type(0);
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp    = this->now();
    msg->header.frame_id = "joint";
    msg->joint_names.push_back(joint_name);
    msg->velocities.push_back(velocity * joint_dir_);
    joint_pub_->publish(std::move(msg));
}

// ─── main keyboard loop ───────────────────────────────────────────────────────
void MoveitServo::run()
{
    running_ = true;

    // Hand the node to the background executor for all future callbacks.
    spin_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(shared_from_this());
        while (running_ && rclcpp::ok()) {
            exec.spin_some(std::chrono::milliseconds(10));
        }
    });

    setup_terminal();

    puts("\n=== MoveIt Servo — Keyboard Control ===");
    puts("  Arrow UP / DOWN    : move X (forward / back)");
    puts("  Arrow LEFT / RIGHT : move Y (left / right)");
    puts("  ';' / '.'          : move Z (up / down)");
    puts("  '[' / ']'          : rotate Z (yaw)");
    puts("  'W'                : use base frame  (world-aligned)");
    puts("  'E'                : use EE frame    (end-effector-aligned)");
    puts("  '1'-'7'            : joint jog (joint1..joint7)");
    puts("  'R'                : reverse joint jog direction");
    puts("  'Q'                : quit");
    puts("=========================================\n");

    char c;
    while (running_ && rclcpp::ok()) {
        if (!read_char(c)) break;

        // Arrow keys arrive as ESC [ A/B/C/D
        if (c == KEYCODE_ESC) {
            char c2 = 0, c3 = 0;
            (void)read(STDIN_FILENO, &c2, 1);
            (void)read(STDIN_FILENO, &c3, 1);
            if (c2 == KEYCODE_BRACKET) {
                switch (c3) {
                    case KEYCODE_UP:
                        RCLCPP_DEBUG(logger_, "UP → +X");
                        publish_twist(linear_speed_, 0, 0);
                        break;
                    case KEYCODE_DOWN:
                        RCLCPP_DEBUG(logger_, "DOWN → -X");
                        publish_twist(-linear_speed_, 0, 0);
                        break;
                    case KEYCODE_LEFT:
                        RCLCPP_DEBUG(logger_, "LEFT → +Y");
                        publish_twist(0, linear_speed_, 0);
                        break;
                    case KEYCODE_RIGHT:
                        RCLCPP_DEBUG(logger_, "RIGHT → -Y");
                        publish_twist(0, -linear_speed_, 0);
                        break;
                    default: break;
                }
            }
            continue;
        }

        switch (c) {
            case KEYCODE_SEMICOLON:
                publish_twist(0, 0, linear_speed_);
                break;
            case KEYCODE_PERIOD:
                publish_twist(0, 0, -linear_speed_);
                break;
            case KEYCODE_RBRACKET:
                publish_twist(0, 0, 0, 0, 0, angular_speed_);
                break;
            case '[':
                publish_twist(0, 0, 0, 0, 0, -angular_speed_);
                break;
            case KEYCODE_W:
                planning_frame_ = base_frame_;
                RCLCPP_INFO(logger_, "Frame → base (%s)", base_frame_.c_str());
                break;
            case KEYCODE_E:
                planning_frame_ = ee_frame_;
                RCLCPP_INFO(logger_, "Frame → end-effector (%s)", ee_frame_.c_str());
                break;
            case KEYCODE_R:
                joint_dir_ *= -1;
                RCLCPP_INFO(logger_, "Joint direction reversed: %+d", joint_dir_);
                break;
            case KEYCODE_Q:
                RCLCPP_INFO(logger_, "Quit requested.");
                running_ = false;
                break;
            default:
                // Joint jog: '1'..'7'
                if (c >= KEYCODE_1 && c <= KEYCODE_7) {
                    int idx = c - KEYCODE_1 + 1;   // 1..7
                    std::string jname = "joint" + std::to_string(idx);
                    RCLCPP_DEBUG(logger_, "Joint jog: %s  dir=%+d", jname.c_str(), joint_dir_);
                    publish_joint_jog(jname, joint_speed_);
                }
                break;
        }
    }

    restore_terminal();
    RCLCPP_INFO(logger_, "MoveitServo keyboard loop exited.");
}

// ─── Autonomous Control (OwnControl) ───────────────────────────────────────

OwnControl::OwnControl(const rclcpp::NodeOptions & options)
: Node("own_control", options)
{
    auto declare_or_get_string = [this](const std::string& name, std::string& var) {
        if (!this->has_parameter(name)) { this->declare_parameter<std::string>(name, var); }
        this->get_parameter(name, var);
    };

    declare_or_get_string("ee_frame", ee_frame_);
    if (ee_frame_.empty()) ee_frame_ = "link_eef";

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_server/delta_twist_cmds", 10);
    switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_server/switch_command_type");

    RCLCPP_INFO(this->get_logger(), "OwnControl node constructed. Call start() to begin autonomous movement.");
}

void OwnControl::switch_command_type(int type)
{
    if (!switch_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "switch_command_type service not ready yet.");
        return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type; // 1 = TWIST

    switch_client_->async_send_request(req,
        [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture f) {
            if (f.get()->success) {
                RCLCPP_INFO(this->get_logger(), "OwnControl successfully switched to TWIST mode.");
            } else {
                RCLCPP_WARN(this->get_logger(), "OwnControl failed to switch command type.");
            }
        }
    );
}

void OwnControl::start()
{
    // Request MoveIt Servo to listen to Twist commands
    switch_command_type(moveit_msgs::srv::ServoCommandType::Request::TWIST);

    start_time_ = this->now();

    // Start timer at 30Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&OwnControl::timer_callback, this));
}

void OwnControl::timer_callback()
{
    double t = (this->now() - start_time_).seconds();

    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = ee_frame_; // Command applied in EE frame

    // "Chicken head" effect: 
    // Position remains perfectly static (linear velocities = 0).
    // The arm moves around to achieve varying orientations.
    msg->twist.linear.x = 0.0;
    msg->twist.linear.y = 0.0;
    msg->twist.linear.z = 0.0;

    // Vary orientation sinusoidally to make the body move continuously
    msg->twist.angular.x = 0.25 * std::sin(t);
    msg->twist.angular.y = 0.25 * std::cos(t);
    msg->twist.angular.z = 0.15 * std::sin(t * 0.5);

    twist_pub_->publish(std::move(msg));
}

}  // namespace xarm_control