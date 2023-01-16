#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class TopicSubscribe01 : public rclcpp::Node
{
public:
  TopicSubscribe01(std::string name) : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
    // 创建一个订阅者订阅话题
    // 1.read config
    // if sensor
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS() , std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));
    // else
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS() , std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));

  }

private:
  // 声明一个订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  void command_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "odom:(%f,%f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
  };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*产生一个的节点*/
  auto node = std::make_shared<TopicSubscribe01>("topic_subscribe_01");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
