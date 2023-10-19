#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class TopicSubscribe01 : public rclcpp::Node
{
public:
  TopicSubscribe01(std::string name) : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(), std::bind(&TopicSubscribe01::odom_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  // 声明一个订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "recv odom->base_footprint tf :(%f,%f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_msg_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg_.pose.pose.position.z = msg->pose.pose.position.z;

    odom_msg_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_msg_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_msg_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom_msg_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  };

public:
  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped transform;
    double seconds = this->now().seconds();
    transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    transform.transform.translation.x = odom_msg_.pose.pose.position.x;
    transform.transform.translation.y = odom_msg_.pose.pose.position.y;
    transform.transform.translation.z = odom_msg_.pose.pose.position.z;
    transform.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
    transform.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
    transform.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
    transform.transform.rotation.w = odom_msg_.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(transform);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*产生一个的节点*/
  auto node = std::make_shared<TopicSubscribe01>("fishbot_bringup");
  /* 运行节点，并检测退出信号*/
  rclcpp::WallRate loop_rate(1000.0);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->publish_tf();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
