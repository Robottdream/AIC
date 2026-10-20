#pragma once
#include <chrono>
#include <cmath>
#include <optional>
#include "behaviortree_cpp_v3/control_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace aic_timed_planner
{
// Two children: compute a candidate path, then follow the committed path.
// No FollowPath tick is permitted before this execution/goal has a valid plan.
class CurrentGoalNavigation : public BT::ControlNode
{
  using Pose=geometry_msgs::msg::PoseStamped;
  using Path=nav_msgs::msg::Path;
  std::optional<Pose> goal_;
  bool have_path_=false;
  std::chrono::steady_clock::time_point next_plan_;

  static bool sameGoal(const Pose & a,const Pose & b) {
    return a.header.frame_id==b.header.frame_id && a.header.stamp==b.header.stamp && a.pose==b.pose;
  }
  static bool matches(const Path & path,const Pose & goal) {
    if(path.poses.empty() || path.header.frame_id!=goal.header.frame_id) return false;
    const auto & end=path.poses.back().pose;
    const double distance=std::hypot(end.position.x-goal.pose.position.x,end.position.y-goal.pose.position.y);
    return std::isfinite(distance) && distance<=.05;
  }
  void clearPath() {
    have_path_=false;goal_.reset();
    setOutput("path",Path{});setOutput("candidate_path",Path{});
  }
public:
  CurrentGoalNavigation(const std::string & name,const BT::NodeConfiguration & config)
  : BT::ControlNode(name,config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<Pose>("goal"),BT::InputPort<double>("hz",5.,"Rolling planning frequency"),
      BT::BidirectionalPort<Path>("candidate_path"),BT::OutputPort<Path>("path")};
  }
  void halt() override {clearPath();BT::ControlNode::halt();}

  BT::NodeStatus tick() override {
    using Status=BT::NodeStatus;
    if(childrenCount()!=2) throw BT::RuntimeError("CurrentGoalNavigation requires planner and follower");
    Pose goal;double hz=5.;
    if(!getInput("goal",goal) || !getInput("hz",hz) || !std::isfinite(hz) || hz<=0)
      throw BT::RuntimeError("Invalid CurrentGoalNavigation goal/rate");
    setStatus(Status::RUNNING);
    if(!goal_ || !sameGoal(*goal_,goal)) {
      // Halt the previous controller AND any in-flight search before starting
      // the new goal. Never allow old blackboard data to open the first gate.
      haltChildren();clearPath();goal_=goal;
      next_plan_=std::chrono::steady_clock::time_point::min();
    }
    auto planner=children_nodes_[0];
    const auto now=std::chrono::steady_clock::now();
    if(!have_path_ || planner->status()==Status::RUNNING || now>=next_plan_) {
      if(planner->status()!=Status::RUNNING) {
        setOutput("candidate_path",Path{});
        next_plan_=now+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1./hz));
      }
      auto planned=planner->executeTick();
      if(planned==Status::SUCCESS) {
        Path candidate;
        if(getInput("candidate_path",candidate) && matches(candidate,goal)) {
          setOutput("path",candidate);have_path_=true;
        }
        haltChild(0);
      } else if(planned==Status::FAILURE) haltChild(0);
      else if(planned!=Status::RUNNING) throw BT::RuntimeError("Planner child returned IDLE");
      if(!have_path_) {
        if(planned==Status::RUNNING) return Status::RUNNING;
        haltChildren();clearPath();return Status::FAILURE;
      }
      // Failed rolling searches retain only the path committed for THIS goal.
    }
    auto followed=children_nodes_[1]->executeTick();
    if(followed==Status::SUCCESS || followed==Status::FAILURE) {
      haltChildren();clearPath();return followed;
    }
    if(followed!=Status::RUNNING) throw BT::RuntimeError("Follower child returned IDLE");
    return Status::RUNNING;
  }
};
}  // namespace aic_timed_planner
