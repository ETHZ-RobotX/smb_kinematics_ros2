#include "smb_kinematics_ros2/smb_differential_drive_controller.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // Required for JointState messages
#include "std_msgs/msg/float64_multi_array.hpp" // Required for Float64MultiArray for publishing wheel velocities

#include <algorithm> // Required for std::find

DifferentialDriveController::DifferentialDriveController()
    : Node("smb_differential_drive_controller")
{
    // Declare and retrieve parameters from the parameter server or config file
    this->declare_parameter<double>("wheel_base", 0.68); //34 22
    this->declare_parameter<double>("wheel_radius", 0.22);
    this->declare_parameter<double>("max_linear_speed", 5.0);
    this->declare_parameter<double>("max_angular_speed", 5.0);
    this->declare_parameter<double>("kinematics_frequency_", 100.0);
    this->declare_parameter<std::string>("input_topic", "/joint_states"); // New parameter for joint states topic
    this->declare_parameter<std::string>("left_wheel_name", "LF_WHEEL_JOINT"); // New parameter for left wheel joint name
    this->declare_parameter<std::string>("right_wheel_name", "RF_WHEEL_JOINT"); // New parameter for right wheel joint name


    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("kinematics_frequency_", kinematics_frequency_);
    this->get_parameter("input_topic", joint_states_input_topic_); // Get joint states input topic
    this->get_parameter("left_wheel_name", left_wheel_name_); // Get left wheel joint name
    this->get_parameter("right_wheel_name", right_wheel_name_); // Get right wheel joint name


    RCLCPP_INFO(this->get_logger(), "Wheel Base: %.2f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "Wheel Radius: %.2f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Max Linear Speed: %.2f", max_linear_speed_);
    RCLCPP_INFO(this->get_logger(), "Max Angular Speed: %.2f", max_angular_speed_);
    RCLCPP_INFO(this->get_logger(), "Kinematics Frequency: %.2f", kinematics_frequency_);
    RCLCPP_INFO(this->get_logger(), "Joint States Input Topic: %s", joint_states_input_topic_.c_str());
    
    // DEBUG: Print the exact values of the parameter-loaded wheel names
    RCLCPP_INFO(this->get_logger(), "Configured Left Wheel Name: '%s'", left_wheel_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Configured Right Wheel Name: '%s'", right_wheel_name_.c_str());


    if (wheel_base_ <= 0.0)
    {
        // RCLCPP_ERROR(this->get_logger(), "Wheel base must be positive");
        throw std::invalid_argument("Wheel base must be positive");
    } 
    if (wheel_radius_ <= 0.0)
    {
        // RCLCPP_ERROR(this->get_logger(), "Wheel radius must be positive");
        throw std::invalid_argument("Wheel radius must be positive");
    }
    if (max_linear_speed_ <= 0.0)
    {
        // RCLCPP_ERROR(this->get_logger(), "Max linear speed must be positive");
        throw std::invalid_argument("Max linear speed must be positive");
    }
    if (max_angular_speed_ <= 0.0)
    {
        // RCLCPP_ERROR(this->get_logger(), "Max angular speed must be positive");
        throw std::invalid_argument("Max angular speed must be positive");
    }
    if (wheel_radius_ > 0.01){
        getMaxWheelSpeed();
        getMaxAngularSpeed();
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Wheel radius must not be zero");
    }
    
    // Create a publisher to publish the wheel velocities (from differential drive calculations)
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_joint_commands", 10);

    // Create a subscription to listen to the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Create a subscription to listen to the joint_states topic
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_input_topic_, 10, std::bind(&DifferentialDriveController::jointStatesCallback, this, std::placeholders::_1));

    // Create a publisher to publish the extracted wheel velocities
    // This will publish: [timestamp, left_angular_vel, right_angular_vel, left_linear_vel, right_linear_vel]
    wheel_velocities_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_velocities", 10);


    // Timer to run computeWheelVelocities at a fixed rate defined by kinematics_frequency_
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kinematics_frequency_)),
        std::bind(&DifferentialDriveController::computeWheelVelocities, this));

    // Initialize dt for the PID controller
    dt_ = 1.0 / kinematics_frequency_;

    last_cmd_vel_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "DifferentialDriveController has been started");
}

DifferentialDriveController::~DifferentialDriveController()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down DifferentialDriveController");
}

void DifferentialDriveController::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    cmd_vel_ = *msg;
    last_cmd_vel_time_ = msg->header.stamp;
}

// Callback function for joint states, publishing in the desired format
void DifferentialDriveController::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    double left_wheel_speed_rps = 0.0;
    double right_wheel_speed_rps = 0.0;

    // Use std::find to get the index of the wheel names
    auto it_left = std::find(msg->name.begin(), msg->name.end(), left_wheel_name_);
    size_t left_idx = std::distance(msg->name.begin(), it_left);

    auto it_right = std::find(msg->name.begin(), msg->name.end(), right_wheel_name_);
    size_t right_idx = std::distance(msg->name.begin(), it_right);

    // Check if both wheel names were found and if their corresponding velocities exist
    bool left_found = (it_left != msg->name.end() && left_idx < msg->velocity.size());
    bool right_found = (it_right != msg->name.end() && right_idx < msg->velocity.size());

    if (left_found)
    {
        left_wheel_speed_rps = msg->velocity[left_idx];
    }

    if (right_found)
    {
        right_wheel_speed_rps = msg->velocity[right_idx];
    }

    
    // Calculate linear wheel speeds
    double left_wheel_speed_ms = left_wheel_speed_rps * wheel_radius_;
    double right_wheel_speed_ms = right_wheel_speed_rps * wheel_radius_;

    // Get timestamp from the message header
    // The timestamp is composed of seconds and nanoseconds
    double time_k = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // Publish the extracted wheel velocities in the desired format
    auto wheel_vel_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    wheel_vel_msg->data = {time_k, left_wheel_speed_rps, right_wheel_speed_rps, left_wheel_speed_ms, right_wheel_speed_ms};
    wheel_velocities_pub_->publish(*wheel_vel_msg);
}


void DifferentialDriveController::computeWheelVelocities()
{
    rclcpp::Time current_time = this->now();

    double linear_requested = cmd_vel_.twist.linear.x;
    double angular_requested = cmd_vel_.twist.angular.z;

    std::clamp(linear_requested, -max_linear_speed_, max_linear_speed_);
    std::clamp(angular_requested, -max_angular_speed_, max_angular_speed_);

    computeKinematics(linear_requested, angular_requested);

    auto left_wheel_sign = std::copysignf(1.0, left_wheel_speed_);
    auto right_wheel_sign = std::copysignf(1.0, right_wheel_speed_);

    double left_over_limit = std::abs(left_wheel_speed_) - max_wheel_speed_;
    double right_over_limit = std::abs(right_wheel_speed_) - max_wheel_speed_;

    if (left_over_limit <= 0.0)
    {
        left_over_limit = 0.0;
    }
    if (right_over_limit <= 0.0)
    {
        right_over_limit = 0.0;
    }

    double alpha = std::max(left_over_limit / max_wheel_speed_, right_over_limit / max_wheel_speed_ );
    alpha = std::clamp(alpha, 0.0, 1.0);

    if (alpha > 0.0)
    {
        computeKinematics(linear_requested - alpha * std::copysignf(1.0, linear_requested), angular_requested);
    }

    if ((current_time - last_cmd_vel_time_).seconds() > 0.1)
    {
        // RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No cmd_vel message received for more than 0.1 seconds");        
        left_wheel_speed_ = 0.0;
        right_wheel_speed_ = 0.0;
    }

    publishWheelVelocities(left_wheel_speed_, right_wheel_speed_);
}

void DifferentialDriveController::computeKinematics(double linear_speed, double angular_speed)
{
    double left_wheel_speed = (linear_speed - angular_speed * wheel_base_ / 2.0) / wheel_radius_;
    double right_wheel_speed = (linear_speed + angular_speed * wheel_base_ / 2.0) / wheel_radius_;
    left_wheel_speed_ = std::clamp(left_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
    right_wheel_speed_ = std::clamp(right_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
}

void DifferentialDriveController::publishWheelVelocities(double left_vel, double right_vel)
{
    auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    msg->data = {this->now().seconds(), left_vel, right_vel};
    joint_command_pub_->publish(*msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
