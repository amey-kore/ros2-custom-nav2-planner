#include "custom_global_planner/custom_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace custom_planner {

void CustomGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  RCLCPP_INFO(rclcpp::get_logger("CustomGlobalPlanner"), "Configuring plugin %s", name_.c_str());
}

void CustomGlobalPlanner::cleanup() {
  RCLCPP_INFO(rclcpp::get_logger("CustomGlobalPlanner"), "Cleaning up plugin %s", name_.c_str());
}

void CustomGlobalPlanner::activate() {
  RCLCPP_INFO(rclcpp::get_logger("CustomGlobalPlanner"), "Activating plugin %s", name_.c_str());
}

void CustomGlobalPlanner::deactivate() {
  RCLCPP_INFO(rclcpp::get_logger("CustomGlobalPlanner"), "Deactivating plugin %s", name_.c_str());
}

nav_msgs::msg::Path CustomGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // Silence the unused parameter warning for the boilerplate
  (void)cancel_checker;

  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = start.header.stamp;
  global_path.header.frame_id = costmap_ros_->getGlobalFrameID();

  // Simple straight-line path for boilerplate
  global_path.poses.push_back(start);
  global_path.poses.push_back(goal);

  return global_path;
}

}  // namespace custom_planner

// This line registers the class as a ROS 2 plugin
PLUGINLIB_EXPORT_CLASS(custom_planner::CustomGlobalPlanner, nav2_core::GlobalPlanner)