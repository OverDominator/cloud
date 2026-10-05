#ifndef FIRE_PERCEPTION_FAR_MARKER_GLOBAL_PLANNER_H
#define FIRE_PERCEPTION_FAR_MARKER_GLOBAL_PLANNER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <mutex>
#include <string>
#include <vector>

namespace fire_perception {

class FarMarkerGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  FarMarkerGlobalPlanner();
  FarMarkerGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  void pathMarkerCallback(const visualization_msgs::Marker::ConstPtr& marker);

  ros::Subscriber path_subscriber_;
  std::mutex path_mutex_;
  std::vector<geometry_msgs::Point> path_points_;
  std::string path_frame_;
  bool initialized_;
  double endpoint_goal_tolerance_;
};

}  // namespace fire_perception

#endif
