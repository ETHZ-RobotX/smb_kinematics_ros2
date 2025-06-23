#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class GlobalToLocalVel : public rclcpp::Node
{
public:
    GlobalToLocalVel()
    : Node("global_to_local_odometry"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        this->declare_parameter("topic_name", "/state_estimation");
        this->declare_parameter("use_ground_truth", false);
        this->declare_parameter("output_topic", "/local_odometry");

        use_ground_truth_ = this->get_parameter("use_ground_truth").as_bool();

        printf("use ground truth: %s\n", use_ground_truth_ ? "true" : "false");

        // if (use_ground_truth_) {
        //     this->set_parameter(rclcpp::Parameter("topic_name", "/odom"));
        // }

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("topic_name").as_string(),
            10,
            std::bind(&GlobalToLocalVel::odometryCallback, this, std::placeholders::_1));

        local_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("output_topic").as_string(),
            10);

        // PointCloud transformation if using ground truth
        if (use_ground_truth_) {
            pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar/points",
                10,
                std::bind(&GlobalToLocalVel::pointcloudCallback, this, std::placeholders::_1)
            );
            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/registered_scan",
                10
            );
        }
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        if (use_ground_truth_) {
            local_odometry_pub_->publish(*msg);
            return; // If using ground truth, just publish the original message, we dont need to transform it
        }

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

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Only transform if using ground truth
        if (!use_ground_truth_) return;

        std::string target_frame = "odom";
        try {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(
                    "odom",  // target frame
                    msg->header.frame_id,  // source frame
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.1)
                );

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);

            pointcloud_pub_->publish(transformed_cloud);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odometry_pub_;

    // For pointcloud transformation
    bool use_ground_truth_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalToLocalVel>());
    rclcpp::shutdown();
    return 0;
}