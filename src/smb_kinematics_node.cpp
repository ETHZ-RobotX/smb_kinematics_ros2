#include "rclcpp/rclcpp.hpp"
#include "smb_kinematics_ros2/smb_differential_drive_controller.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
