#ifndef SMB_DIFFERENTIAL_DRIVE_CONTROLLER_HPP
#define SMB_DIFFERENTIAL_DRIVE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>


class DifferentialDriveController : public rclcpp::Node
{
public:
    DifferentialDriveController();
    ~DifferentialDriveController();

private:
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void computeWheelVelocities();
    void computeKinematics(double linear_speed, double angular_speed);
    void publishWheelVelocities(double left_vel, double right_vel);

    void getMaxWheelSpeed(){ max_wheel_speed_ = max_linear_speed_ / wheel_radius_; }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    // Robot Parameters 
    double wheel_base_{1.0};
    double wheel_radius_{1.0};

    // Limits
    double max_linear_speed_{0.0};
    double max_angular_speed_{0.0};
    double max_linear_accel_{0.0};
    double max_angular_accel_{0.0};
    double max_wheel_speed_{0.0};

    // Command velocity
    geometry_msgs::msg::TwistStamped cmd_vel_;
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time last_odom_time_;

    double left_wheel_speed_ = 0.0;
    double right_wheel_speed_ = 0.0;

    // PID parameters & Control variables
    double linear_kp_, linear_ki_, linear_kd_;
    double angular_kp_, angular_ki_, angular_kd_;
    double linear_integral_, linear_previous_error_;
    double angular_integral_, angular_previous_error_;
    double kinematics_frequency_{100}; // 100 Hz
    rclcpp::Time last_control_time_;
    double dt_{1.0};
};

#endif // SMB_DIFFERENTIAL_DRIVE_CONTROLLER_HPP
