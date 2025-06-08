#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GlobalToLocalVel : public rclcpp::Node
{
public:
    GlobalToLocalVel()
    : Node("global_to_local_odometry")
    {
        this->declare_parameter("topic_name", "/dlio/odom_node/odom");
        this->declare_parameter("output_topic", "/local_odometry");


        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("topic_name").as_string(),
            10,
            std::bind(&GlobalToLocalVel::odometryCallback, this, std::placeholders::_1));

        local_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("output_topic").as_string(),
            10);
    }

private:

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Vector3 linear_velocity(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);

        tf2::Vector3 angular_velocity(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z);

        tf2::Vector3 local_linear_velocity = tf2::quatRotate(quat.inverse(), linear_velocity);
        tf2::Vector3 local_angular_velocity = tf2::quatRotate(quat.inverse(), angular_velocity);

        nav_msgs::msg::Odometry local_odometry;
        local_odometry.header = msg->header;
        local_odometry.child_frame_id = msg->child_frame_id;
        local_odometry.pose.pose = msg->pose.pose;
        local_odometry.twist.twist.linear.x = local_linear_velocity.x();
        local_odometry.twist.twist.linear.y = local_linear_velocity.y();
        local_odometry.twist.twist.linear.z = local_linear_velocity.z();
        local_odometry.twist.twist.angular.x = local_angular_velocity.x();
        local_odometry.twist.twist.angular.y = local_angular_velocity.y();
        local_odometry.twist.twist.angular.z = local_angular_velocity.z();

        local_odometry_pub_->publish(local_odometry);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odometry_pub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalToLocalVel>());
    rclcpp::shutdown();
    return 0;
}