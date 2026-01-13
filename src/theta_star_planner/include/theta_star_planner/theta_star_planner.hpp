#ifndef THETA_STAR_PLANNER_HPP
#define THETA_STAR_PLANNER_HPP

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>

namespace theta_star_planner
{

class ThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  ThetaStarPlanner() = default;
  ~ThetaStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Node
  {
    int x, y;
    double g, f;
    Node * parent;
  };

  bool isCellFree(int x, int y);
  bool lineOfSight(int x0, int y0, int x1, int y1);
  double heuristic(int x0, int y0, int x1, int y1);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;
};

}  // namespace theta_star_planner

#endif

