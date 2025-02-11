#include "smb_kinematics/smb_differential_drive_controller.hpp"

DifferentialDriveController::DifferentialDriveController()
    : Node("smb_differential_drive_controller"),
      linear_integral_(0.0), linear_previous_error_(0.0),
      angular_integral_(0.0), angular_previous_error_(0.0)
{
    // Declare and retrieve parameters from the parameter server or config file
    this->declare_parameter<double>("wheel_base", 0.6);
    this->declare_parameter<double>("wheel_radius", 0.15);
    this->declare_parameter<double>("max_linear_speed", 1.2);
    this->declare_parameter<double>("max_angular_speed", 1.0);
    this->declare_parameter<double>("linear_kp", 1.0);
    this->declare_parameter<double>("linear_ki", 0.0);
    this->declare_parameter<double>("linear_kd", 0.0);
    this->declare_parameter<double>("angular_kp", 1.0);
    this->declare_parameter<double>("angular_ki", 0.0);
    this->declare_parameter<double>("angular_kd", 0.0);
    this->declare_parameter<double>("controller_frequency_", 100.0);

    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("linear_kp", linear_kp_);
    this->get_parameter("linear_ki", linear_ki_);
    this->get_parameter("linear_kd", linear_kd_);
    this->get_parameter("angular_kp", angular_kp_);
    this->get_parameter("angular_ki", angular_ki_);
    this->get_parameter("angular_kd", angular_kd_);
    this->get_parameter("controller_frequency_", controller_frequency_);

    RCLCPP_INFO(this->get_logger(), "Wheel Base: %.2f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "Wheel Radius: %.2f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Max Linear Speed: %.2f", max_linear_speed_);
    RCLCPP_INFO(this->get_logger(), "Max Angular Speed: %.2f", max_angular_speed_);
    RCLCPP_INFO(this->get_logger(), "Linear PID: %.2f, %.2f, %.2f", linear_kp_, linear_ki_, linear_kd_);
    RCLCPP_INFO(this->get_logger(), "Angular PID: %.2f, %.2f, %.2f", angular_kp_, angular_ki_, angular_kd_);
    RCLCPP_INFO(this->get_logger(), "Controller Frequency: %.2f", controller_frequency_);
    
    calculateMaxWheelSpeed();

    // Create a publisher to publish the wheel velocities
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_joint_commands", 10);
    // LH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("LH_wheel_joint_velocity_gazebo", 10);
    // RH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("RH_wheel_joint_velocity_gazebo", 10);
    // RF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("RF_wheel_joint_velocity_gazebo", 10);
    // LF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("LF_wheel_joint_velocity_gazebo", 10);

    // Create a subscription to listen to the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "state_estimation", 10, std::bind(&DifferentialDriveController::odomCallback, this, std::placeholders::_1));

    // Timer to run computeWheelVelocities at a fixed rate defined by controller_frequency_
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / controller_frequency_)),
        std::bind(&DifferentialDriveController::computeWheelVelocities, this));

    // Initialize dt for the PID controller
    dt_ = 1.0 / controller_frequency_;

    last_cmd_vel_time_ = this->now();
    last_odom_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "DifferentialDriveController has been started");
}

DifferentialDriveController::~DifferentialDriveController()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down DifferentialDriveController");
}

void DifferentialDriveController::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    cmd_vel_ = *msg;
    last_cmd_vel_time_ = this->now();

    // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f angular=%.2f", msg->linear.x, msg->angular.z);
}

void DifferentialDriveController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = *msg;
    last_odom_time_ = this->now();
    received_odom_ = true;
}

void DifferentialDriveController::computeWheelVelocities()
{
    // double left_vel_ = 0.0;
    // double right_vel_ = 0.0;

    rclcpp::Time current_time = this->now();

    if ((current_time - last_cmd_vel_time_).seconds() > 0.2 || (current_time - last_odom_time_).seconds() > 0.2)
    {
        // RCLCPP_WARN(this->get_logger(), "Timeout detected. Setting velocities to zero.");
        cmd_vel_.twist.linear.x = 0.0;
        cmd_vel_.twist.angular.z = 0.0;
        odom_.twist.twist.linear.x = 0.0;
        odom_.twist.twist.angular.z = 0.0;
    }

    if (!received_odom_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No odometry message received yet");
        return;
    }

    double current_linear = odom_.twist.twist.linear.x;
    double current_angular = odom_.twist.twist.angular.z;

    RCLCPP_INFO(this->get_logger(), "Current velocities: linear=%.2f angular=%.2f", current_linear, current_angular);

    double linear_output = cmd_vel_.twist.linear.x + pidControl(cmd_vel_.twist.linear.x, current_linear, linear_integral_, linear_previous_error_, linear_kp_, linear_ki_, linear_kd_);
    double angular_output = cmd_vel_.twist.angular.z + pidControl(cmd_vel_.twist.angular.z, current_angular, angular_integral_, angular_previous_error_, angular_kp_, angular_ki_, angular_kd_);

    RCLCPP_ERROR(this->get_logger(), "Linear output: %.2f", linear_output);
    RCLCPP_ERROR(this->get_logger(), "Angular output: %.2f", angular_output);
    RCLCPP_ERROR(this->get_logger(), "gains: %.2f, %.2f, %.2f", linear_kp_, linear_ki_, linear_kd_);
    RCLCPP_ERROR(this->get_logger(), "gains: %.2f, %.2f, %.2f", angular_kp_, angular_ki_, angular_kd_);

    left_vel_ = (linear_output - angular_output * wheel_base_ / 2.0) / wheel_radius_;
    right_vel_ = (linear_output + angular_output * wheel_base_ / 2.0) / wheel_radius_;

    left_vel_ = std::clamp(left_vel_, -max_wheel_speed_, max_wheel_speed_);
    right_vel_ = std::clamp(right_vel_, -max_wheel_speed_, max_wheel_speed_);

    RCLCPP_ERROR(this->get_logger(), "max_wheel_speed_: %.2f", max_wheel_speed_);
    RCLCPP_ERROR(this->get_logger(), "left_vel_: %.2f", left_vel_);
    RCLCPP_ERROR(this->get_logger(), "right_vel_: %.2f", right_vel_);
    RCLCPP_ERROR(this->get_logger(), "wheel radius and base: %.2f, %.2f", wheel_radius_, wheel_base_);

    RCLCPP_INFO(this->get_logger(), "Computed wheel speeds: left=%.2f right=%.2f", left_vel_, right_vel_);

    // left_vel_ = 1.0;
    // right_vel_ = -1.0;

    publishWheelVelocities(left_vel_, right_vel_);
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

    // auto LH_msg = std::make_shared<std_msgs::msg::Float64>();
    // auto RH_msg = std::make_shared<std_msgs::msg::Float64>();
    // auto RF_msg = std::make_shared<std_msgs::msg::Float64>();
    // auto LF_msg = std::make_shared<std_msgs::msg::Float64>();

    // LH_msg->data = left_vel;
    // RH_msg->data = right_vel;
    // RF_msg->data = right_vel;
    // LF_msg->data = left_vel;

    // LH_joint_velocity_pub_->publish(*LH_msg);
    // RH_joint_velocity_pub_->publish(*RH_msg);
    // RF_joint_velocity_pub_->publish(*RF_msg);
    // LF_joint_velocity_pub_->publish(*LF_msg);

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
