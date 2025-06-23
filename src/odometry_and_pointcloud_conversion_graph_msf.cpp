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

class OdomPointCloudConversionGraphMsf : public rclcpp::Node
{
public:
    OdomPointCloudConversionGraphMsf()
    : Node("global_to_local_odometry"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        this->declare_parameter("topic_name", "/graph_msf/est_odometry_world_imu");
        this->declare_parameter("use_ground_truth", false);
        this->declare_parameter("output_topic", "/state_estimation");

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("topic_name").as_string(),
            10,
            std::bind(&OdomPointCloudConversionGraphMsf::odometryCallback, this, std::placeholders::_1));

        state_estimation_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("output_topic").as_string(),
            10);

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar/points",
            10,
            std::bind(&OdomPointCloudConversionGraphMsf::pointcloudCallback, this, std::placeholders::_1)
        );
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/registered_scan",
            10
        );
    }

private:

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(
                    msg->child_frame_id,  
                    "rslidar",  
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.1)
                );

            nav_msgs::msg::Odometry transformed_odometry;
            tf2::doTransform(msg->pose.pose, transformed_odometry.pose.pose, transform_stamped);
            
            transformed_odometry.child_frame_id = "rslidar";
            transformed_odometry.header.frame_id = "odom";

            state_estimation_pub_->publish(transformed_odometry);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odometry: %s", ex.what());
        }
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(
                    "world_graph_msf",  
                    msg->header.frame_id, 
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.1)
                );

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);

            transformed_cloud.header.frame_id = "odom";

            pointcloud_pub_->publish(transformed_cloud);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_estimation_pub_;

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
    rclcpp::spin(std::make_shared<OdomPointCloudConversionGraphMsf>());
    rclcpp::shutdown();
    return 0;
}