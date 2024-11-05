#include "smb_differential_drive_controller.hpp"

DifferentialDriveController::DifferentialDriveController()
    : Node("smb_differential_drive_controller"),
      linear_integral_(0.0), linear_previous_error_(0.0),
      angular_integral_(0.0), angular_previous_error_(0.0)
{
    // Declare and retrieve parameters from the parameter server or config file
    this->declare_parameter<double>("wheel_base", 0.6);
    this->declare_parameter<double>("wheel_radius", 0.15);
    this->declare_parameter<double>("max_speed", 1.2);
    this->declare_parameter<double>("linear_kp", 1.0);
    this->declare_parameter<double>("linear_ki", 0.0);
    this->declare_parameter<double>("linear_kd", 0.0);
    this->declare_parameter<double>("angular_kp", 1.0);
    this->declare_parameter<double>("angular_ki", 0.0);
    this->declare_parameter<double>("angular_kd", 0.0);
    this->declare_parameter<double>("controller_frequency_", 100.0);

    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("linear_kp", linear_kp_);
    this->get_parameter("linear_ki", linear_ki_);
    this->get_parameter("linear_kd", linear_kd_);
    this->get_parameter("angular_kp", angular_kp_);
    this->get_parameter("angular_ki", angular_ki_);
    this->get_parameter("angular_kd", angular_kd_);
    this->get_parameter("controller_frequency_", controller_frequency_);

    RCLCPP_INFO(this->get_logger(), "Wheel Base: %.2f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "Wheel Radius: %.2f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Max Speed: %.2f", max_speed_);
    RCLCPP_INFO(this->get_logger(), "Linear PID: %.2f, %.2f, %.2f", linear_kp_, linear_ki_, linear_kd_);
    RCLCPP_INFO(this->get_logger(), "Angular PID: %.2f, %.2f, %.2f", angular_kp_, angular_ki_, angular_kd_);
    RCLCPP_INFO(this->get_logger(), "Controller Frequency: %.2f", controller_frequency_);

    // Create a publisher to publish the wheel velocities
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_joint_commands", 10);

    // Create a subscription to listen to the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Timer to run computeWheelVelocities at a fixed rate defined by controller_frequency_
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / controller_frequency_)),
        std::bind(&DifferentialDriveController::computeWheelVelocities, this));

    // Initialize dt for the PID controller
    dt_ = 1.0 / controller_frequency_;


    RCLCPP_INFO(this->get_logger(), "DifferentialDriveController has been started");
}

DifferentialDriveController::~DifferentialDriveController()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down DifferentialDriveController");
}

void DifferentialDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = *msg;
    last_cmd_vel_time_ = this->now();
}

void DifferentialDriveController::computeWheelVelocities()
{
    double left_vel = 0.0;
    double right_vel = 0.0;

    double current_linear = cmd_vel_.linear.x;
    double current_angular = cmd_vel_.angular.z;

    double linear_output = pidControl(current_linear, cmd_vel_.linear.x, linear_integral_, linear_previous_error_, linear_kp_, linear_ki_, linear_kd_);
    double angular_output = pidControl(current_angular, cmd_vel_.angular.z, angular_integral_, angular_previous_error_, angular_kp_, angular_ki_, angular_kd_);

    left_vel = (linear_output - angular_output * wheel_base_ / 2.0) / wheel_radius_;
    right_vel = (linear_output + angular_output * wheel_base_ / 2.0) / wheel_radius_;

    left_vel = std::clamp(left_vel, -max_speed_, max_speed_);
    right_vel = std::clamp(right_vel, -max_speed_, max_speed_);

    publishWheelVelocities(left_vel, right_vel);
}

double DifferentialDriveController::pidControl(double target, double current, double &integral, double &previous_error, double kp, double ki, double kd)
{
    double error = target - current;
    integral += error;
    double derivative = error - previous_error;
    previous_error = error;

    return kp * error + ki * integral + kd * derivative;
}

void DifferentialDriveController::publishWheelVelocities(double left_vel, double right_vel)
{
    auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    msg->data = {left_vel, right_vel};
    joint_command_pub_->publish(*msg);
}
