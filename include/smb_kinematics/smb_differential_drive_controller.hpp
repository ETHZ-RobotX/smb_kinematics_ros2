#ifndef SMB_DIFFERENTIAL_DRIVE_CONTROLLER_HPP
#define SMB_DIFFERENTIAL_DRIVE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>


class DifferentialDriveController : public rclcpp::Node
{
public:
    DifferentialDriveController();
    ~DifferentialDriveController();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void computeWheelVelocities();
    void publishWheelVelocities(double left_vel, double right_vel);
    double pidControl(double target, double current, double &integral, double &previous_error, double kp, double ki, double kd);

    void calculateMaxWheelSpeed(){ max_wheel_speed_ = max_linear_speed_ / wheel_radius_; }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr LH_joint_velocity_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr RH_joint_velocity_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr RF_joint_velocity_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr LF_joint_velocity_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Robot Parameters 
    double wheel_base_;
    double wheel_radius_;

    // Limits
    double max_linear_speed_;
    double max_angular_speed_;
    double max_linear_accel_;
    double max_angular_accel_;
    double max_wheel_speed_;

    // Command velocity
    geometry_msgs::msg::Twist cmd_vel_;
    rclcpp::Time last_cmd_vel_time_;

    double left_vel_ = 0.0;
    double right_vel_ = 0.0;

    // PID parameters & Control variables
    double linear_kp_, linear_ki_, linear_kd_;
    double angular_kp_, angular_ki_, angular_kd_;
    double linear_integral_, linear_previous_error_;
    double angular_integral_, angular_previous_error_;
    double controller_frequency_{100}; // 100 Hz
    rclcpp::Time last_control_time_;
    double dt_{1.0};
};

#endif // SMB_DIFFERENTIAL_DRIVE_CONTROLLER_HPP
