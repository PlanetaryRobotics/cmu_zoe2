#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomToBaseLinkBroadcaster : public rclcpp::Node
{
public:
    OdomToBaseLinkBroadcaster()
        : Node("odom_tf_broadcaster")
    {
        // Create a transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscribe to the /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomToBaseLinkBroadcaster::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;

        // Header information
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        // Translation (position from odometry)
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // Rotation (orientation from odometry)
        transform.transform.rotation = msg->pose.pose.orientation;

        // Publish the transform
        tf_broadcaster_->sendTransform(transform);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToBaseLinkBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
