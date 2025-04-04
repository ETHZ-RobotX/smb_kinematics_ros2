#include "smb_kinematics_ros2/smb_differential_drive_controller.hpp"

DifferentialDriveController::DifferentialDriveController()
    : Node("smb_differential_drive_controller")
{
    // Declare and retrieve parameters from the parameter server or config file
    this->declare_parameter<double>("wheel_base", 0.68); //34 22
    this->declare_parameter<double>("wheel_radius", 0.22);
    this->declare_parameter<double>("max_linear_speed", 1.0);
    this->declare_parameter<double>("max_angular_speed", 1.0);
    this->declare_parameter<double>("kinematics_frequency_", 100.0);

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
    this->get_parameter("kinematics_frequency_", kinematics_frequency_);

    RCLCPP_INFO(this->get_logger(), "Wheel Base: %.2f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "Wheel Radius: %.2f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Max Linear Speed: %.2f", max_linear_speed_);
    RCLCPP_INFO(this->get_logger(), "Max Angular Speed: %.2f", max_angular_speed_);
    RCLCPP_INFO(this->get_logger(), "Kinematics Frequency: %.2f", kinematics_frequency_);

    if (wheel_base_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Wheel base must be positive");
        throw std::invalid_argument("Wheel base must be positive");
    } 
    if (wheel_radius_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Wheel radius must be positive");
        throw std::invalid_argument("Wheel radius must be positive");
    }
    if (max_linear_speed_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Max linear speed must be positive");
        throw std::invalid_argument("Max linear speed must be positive");
    }
    if (max_angular_speed_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Max angular speed must be positive");
        throw std::invalid_argument("Max angular speed must be positive");
    }
    if (wheel_radius_ > 0.01){
        getMaxWheelSpeed();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Wheel radius must not be zero");
    }
    
    // Create a publisher to publish the wheel velocities
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_joint_commands", 10);

    // Create a subscription to listen to the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

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

void DifferentialDriveController::computeWheelVelocities()
{
    rclcpp::Time current_time = this->now();

    if ((current_time - last_cmd_vel_time_).seconds() > 0.1)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No cmd_vel message received for more than 0.1 seconds");        
        cmd_vel_.twist.linear.x = 0.0;
        cmd_vel_.twist.angular.z = 0.0;
    }

    double linear_requested = cmd_vel_.twist.linear.x;
    double angular_requested = cmd_vel_.twist.angular.z;

    computeKinematics(linear_requested, angular_requested);

    double alpha = std::max(std::abs(left_wheel_speed_) - max_wheel_speed_, std::abs(right_wheel_speed_) - max_wheel_speed_);

    if (alpha > 0.0)
    {
        computeKinematics(linear_requested - alpha * std::copysignf(1.0, linear_requested), angular_requested);
    }

    // Safety
    left_wheel_speed_ = std::clamp(left_wheel_speed_, -max_wheel_speed_, max_wheel_speed_);
    right_wheel_speed_ = std::clamp(right_wheel_speed_, -max_wheel_speed_, max_wheel_speed_);

    publishWheelVelocities(left_wheel_speed_, right_wheel_speed_);
}

void DifferentialDriveController::computeKinematics(double linear_speed, double angular_speed)
{
    left_wheel_speed_ = (linear_speed - angular_speed * wheel_base_ / 2.0) / wheel_radius_;
    right_wheel_speed_ = (linear_speed + angular_speed * wheel_base_ / 2.0) / wheel_radius_;
}

void DifferentialDriveController::publishWheelVelocities(double left_vel, double right_vel)
{
    auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    msg->data = {left_vel, right_vel};
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
