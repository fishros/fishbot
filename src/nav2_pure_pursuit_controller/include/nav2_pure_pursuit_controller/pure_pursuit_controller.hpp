#ifndef NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_pure_pursuit_controller {

class PurePursuitController : public nav2_core::Controller {
 public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;
  /**
   * @brief 配置控制器
   *
   * @param parent 生命周期节点指针
   * @param name 节点名称
   * @param tf tf操作句柄
   * @param costmap_ros 代价地图
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  /**
   * @brief
   *
   * @param pose
   * @param velocity 速度指令
   * @return geometry_msgs::msg::TwistStamped
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) override;
  /**
   * @brief Set the Plan object
   *
   * @param path
   */
  void setPlan(const nav_msgs::msg::Path &path) override;
  /**
   * @brief Set the Speed Limit object
   *
   * @param speed_limit
   * @param percentage
   */
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

 protected:
  nav_msgs::msg::Path transformGlobalPlan(
      const geometry_msgs::msg::PoseStamped &pose);

  bool transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                     const std::string frame,
                     const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose,
                     const rclcpp::Duration &transform_tolerance) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  rclcpp::Duration transform_tolerance_{0, 0};

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
      global_pub_;
};

}  // namespace nav2_pure_pursuit_controller

#endif  // NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
