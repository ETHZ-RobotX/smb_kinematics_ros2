#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel()
    : Node("joy_to_cmd_vel")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyToCmdVel::joyCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        max_linear_ = 1.0;
        max_angular_ = 1.0;
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::TwistStamped();
        twist.header.stamp = this->now();

        // Left stick: axes[1] = forward/back, axes[0] = left/right
        if (msg->axes.size() >= 2) {
            twist.twist.linear.x = max_linear_ * msg->axes[1];
            twist.twist.angular.z = max_angular_ * msg->axes[2];
        } else {
            twist.twist.linear.x = 0.0;
            twist.twist.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    double max_linear_;
    double max_angular_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}