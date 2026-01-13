#include "theta_star_planner/theta_star_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <queue>
#include <cmath>

namespace theta_star_planner
{



void ThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
}

void ThetaStarPlanner::cleanup() {}
void ThetaStarPlanner::activate() {}
void ThetaStarPlanner::deactivate() {}

bool ThetaStarPlanner::isCellFree(int x, int y)
{
  unsigned char cost = costmap_->getCost(x, y);
  return cost < 254;
}

bool ThetaStarPlanner::lineOfSight(int x0, int y0, int x1, int y1)
{
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);

  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;

  int err = dx - dy;

  while (true)
  {
    if (!isCellFree(x0, y0))
      return false;

    if (x0 == x1 && y0 == y1)
      return true;

    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx)  { err += dx; y0 += sy; }
  }
}

double ThetaStarPlanner::heuristic(int x0, int y0, int x1, int y1)
{
  return std::hypot(x1 - x0, y1 - y0);
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  unsigned int sx, sy, gx, gy;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy);

  struct Node
  {
    int x, y;
    double g, f;
    Node* parent;
  };

  auto cmp = [](Node* a, Node* b) { return a->f > b->f; };
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

  std::vector<std::vector<bool>> closed(costmap_->getSizeInCellsX(),
                                        std::vector<bool>(costmap_->getSizeInCellsY(), false));

  Node* start_node = new Node{(int)sx, (int)sy, 0, heuristic(sx, sy, gx, gy), nullptr};
  open.push(start_node);

  Node* goal_node = nullptr;

  int dx[8] = {1,1,0,-1,-1,-1,0,1};
  int dy[8] = {0,1,1,1,0,-1,-1,-1};

  while (!open.empty())
  {
    Node* current = open.top();
    open.pop();

    if (current->x == (int)gx && current->y == (int)gy)
    {
      goal_node = current;
      break;
    }

    closed[current->x][current->y] = true;

    for (int i = 0; i < 8; i++)
    {
      int nx = current->x + dx[i];
      int ny = current->y + dy[i];

      if (nx < 0 || ny < 0 || nx >= (int)costmap_->getSizeInCellsX() || ny >= (int)costmap_->getSizeInCellsY())
        continue;

      if (!isCellFree(nx, ny) || closed[nx][ny])
        continue;

      double new_g;
      Node* parent = current;

      if (current->parent && lineOfSight(current->parent->x, current->parent->y, nx, ny))
      {
        parent = current->parent;
        new_g = parent->g + heuristic(parent->x, parent->y, nx, ny);
      }
      else
      {
        new_g = current->g + heuristic(current->x, current->y, nx, ny);
      }

      Node* neighbor = new Node{nx, ny, new_g, new_g + heuristic(nx, ny, gx, gy), parent};
      open.push(neighbor);
    }
  }

  while (goal_node)
  {
    geometry_msgs::msg::PoseStamped pose;
    double wx, wy;
    costmap_->mapToWorld(goal_node->x, goal_node->y, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    goal_node = goal_node->parent;
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return path;
}

}  

PLUGINLIB_EXPORT_CLASS(theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
