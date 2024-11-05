#include "rclcpp/rclcpp.hpp"
#include "smb_kinematics/differential_drive_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveController>();

    // Optionally, set the maximum speed if needed
    node->setMaxSpeed(1.0);  // Set max speed or get it from parameter server

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
