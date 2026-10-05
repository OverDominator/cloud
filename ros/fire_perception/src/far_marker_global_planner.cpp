#include "fire_perception/far_marker_global_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>

namespace fire_perception {
namespace {

double Distance2D(const geometry_msgs::Point& first,
                  const geometry_msgs::Point& second) {
  return std::hypot(first.x - second.x, first.y - second.y);
}

geometry_msgs::Point PosePoint(const geometry_msgs::PoseStamped& pose) {
  return pose.pose.position;
}

}  // namespace

FarMarkerGlobalPlanner::FarMarkerGlobalPlanner()
    : initialized_(false), endpoint_goal_tolerance_(8.0) {}

FarMarkerGlobalPlanner::FarMarkerGlobalPlanner(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : FarMarkerGlobalPlanner() {
  initialize(name, costmap_ros);
}

void FarMarkerGlobalPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* /*costmap_ros*/) {
  if (initialized_) return;

  ros::NodeHandle private_nh("~/" + name);
  std::string path_topic;
  private_nh.param<std::string>("far_path_topic", path_topic,
                                "/viz_path_topic");
  private_nh.param("endpoint_goal_tolerance", endpoint_goal_tolerance_, 8.0);
  path_subscriber_ = private_nh.subscribe(
      path_topic, 5, &FarMarkerGlobalPlanner::pathMarkerCallback, this);
  initialized_ = true;
  ROS_INFO("FarMarkerGlobalPlanner listening to %s", path_topic.c_str());
}

void FarMarkerGlobalPlanner::pathMarkerCallback(
    const visualization_msgs::Marker::ConstPtr& marker) {
  if (marker->ns != "global_path" ||
      marker->type != visualization_msgs::Marker::LINE_STRIP) {
    return;
  }

  std::lock_guard<std::mutex> lock(path_mutex_);
  if (marker->action == visualization_msgs::Marker::DELETE ||
      marker->action == visualization_msgs::Marker::DELETEALL ||
      marker->points.size() < 2) {
    path_points_.clear();
    return;
  }
  path_points_ = marker->points;
  path_frame_ = marker->header.frame_id.empty() ? "map" : marker->header.frame_id;
}

bool FarMarkerGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!initialized_) {
    ROS_ERROR_THROTTLE(2.0, "FarMarkerGlobalPlanner is not initialized");
    return false;
  }

  std::vector<geometry_msgs::Point> points;
  std::string frame;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    points = path_points_;
    frame = path_frame_;
  }

  if (points.size() < 2) {
    ROS_WARN_THROTTLE(2.0, "Waiting for a FAR global path marker");
    return false;
  }

  const geometry_msgs::Point start_point = PosePoint(start);
  const geometry_msgs::Point goal_point = PosePoint(goal);
  if (Distance2D(points.back(), start_point) <
      Distance2D(points.front(), start_point)) {
    std::reverse(points.begin(), points.end());
  }

  if (Distance2D(points.back(), goal_point) > endpoint_goal_tolerance_) {
    ROS_WARN_THROTTLE(
        2.0,
        "Cached FAR path endpoint is %.2f m from the requested goal; waiting for update",
        Distance2D(points.back(), goal_point));
    return false;
  }

  const std::string output_frame = goal.header.frame_id.empty() ? frame : goal.header.frame_id;
  auto append_pose = [&](const geometry_msgs::Point& point) {
    if (!plan.empty() &&
        Distance2D(plan.back().pose.position, point) < 0.05) {
      return;
    }
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = output_frame;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  };

  append_pose(start_point);
  for (const auto& point : points) append_pose(point);
  append_pose(goal_point);
  if (plan.size() < 2) return false;

  for (std::size_t index = 0; index + 1 < plan.size(); ++index) {
    const double yaw = std::atan2(
        plan[index + 1].pose.position.y - plan[index].pose.position.y,
        plan[index + 1].pose.position.x - plan[index].pose.position.x);
    plan[index].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }
  plan.back().pose.orientation = goal.pose.orientation;

  ROS_INFO_THROTTLE(2.0, "Forwarding FAR global plan with %zu poses", plan.size());
  return true;
}

}  // namespace fire_perception

PLUGINLIB_EXPORT_CLASS(fire_perception::FarMarkerGlobalPlanner,
                       nav_core::BaseGlobalPlanner)
