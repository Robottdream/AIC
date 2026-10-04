// Copyright 2026 AIC contributors. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bot_navigation
{
// Follow the ordered polyline rather than trading endpoint PathDist against a
// distant local goal. The latter can prefer a chord, or equal-cost zero motion.
class PathFollowCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
    auto param = [&](const std::string & key, double value) {
        if (!node->has_parameter(prefix + key)) {node->declare_parameter(prefix + key, value);}
        return node->get_parameter(prefix + key).as_double();
      };
    lookahead_ = param("lookahead_distance", 0.65);
    control_horizon_ = param("control_horizon", 0.6);
    corridor_ = param("max_prefix_deviation", 0.12);
    chord_limit_ = param("max_path_chord_deviation", 0.0);
    if (lookahead_ <= 0 || control_horizon_ <= 0 || corridor_ <= 0) {
      throw std::invalid_argument("PathFollow distances and horizon must be positive");
    }
  }

  bool prepare(const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D & path) override
  {
    points_ = path.poses;
    if (points_.empty()) {return false;}
    segments_.clear();
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const double dx = points_[i + 1].x - points_[i].x;
      const double dy = points_[i + 1].y - points_[i].y;
      const double squared = dx * dx + dy * dy;
      segments_.push_back({dx, dy, squared > 1e-12 ? 1.0 / squared : 0.0});
    }
    const auto nearest = project(pose);
    initial_distance_ = nearest.distance;
    double remaining = lookahead_;
    target_ = nearest.point;
    std::vector<geometry_msgs::msg::Pose2D> walked{nearest.point};
    for (size_t i = nearest.segment; i + 1 < points_.size(); ++i) {
      const auto & end = points_[i + 1];
      const double length = std::hypot(end.x - target_.x, end.y - target_.y);
      auto candidate = end;
      if (length >= remaining && length > 1e-9) {
        const double fraction = remaining / length;
        candidate.x = target_.x + fraction * (end.x - target_.x);
        candidate.y = target_.y + fraction * (end.y - target_.y);
      }
      // A long straight-road target must not turn into a chord across a bend.
      // Shorten it when intermediate reference points leave the chord corridor.
      if (chord_limit_ > 0 && walked.size() > 1) {
        const auto & start = nearest.point;
        const double dx = candidate.x - start.x, dy = candidate.y - start.y;
        const double squared = dx * dx + dy * dy;
        double deviation = 0;
        for (const auto & point : walked) {
          const double f = squared > 1e-12 ? std::max(0.0, std::min(1.0,
            ((point.x - start.x) * dx + (point.y - start.y) * dy) / squared)) : 0;
          deviation = std::max(deviation, std::hypot(point.x - start.x - f * dx,
            point.y - start.y - f * dy));
        }
        if (deviation > chord_limit_) {break;}
      }
      target_ = candidate;
      walked.push_back(candidate);
      if (length >= remaining) {break;}
      remaining -= length;
    }
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (trajectory.poses.empty() || trajectory.time_offsets.empty()) {
      throw dwb_core::IllegalTrajectoryException(name_, "Missing trajectory samples");
    }
    double sum = 0, peak = 0;
    auto control_pose = trajectory.poses.front();
    double previous_time = 0;
    for (size_t i = 0; i < trajectory.poses.size(); ++i) {
      const auto & p = trajectory.poses[i];
      const double t = rclcpp::Duration(trajectory.time_offsets[
          std::min(i, trajectory.time_offsets.size() - 1)]).seconds();
      const double distance = project(p).distance;
      sum += distance;
      peak = std::max(peak, distance);
      // Allow a robot already off the route to return; do not reject its start.
      // Keep only a short prefix strict: the full 1.8 s constant-twist rollout
      // cannot bend, although the controller replans every 50 ms.
      if (t <= control_horizon_ + 1e-6 && distance >
        std::max(corridor_, initial_distance_ + 0.03))
      {
        throw dwb_core::IllegalTrajectoryException(name_, "Corner-cutting control prefix");
      }
      if (previous_time < control_horizon_ && t >= control_horizon_) {
        const auto & previous = trajectory.poses[i == 0 ? 0 : i - 1];
        const double fraction = t > previous_time ?
          (control_horizon_ - previous_time) / (t - previous_time) : 1;
        control_pose.x = previous.x + fraction * (p.x - previous.x);
        control_pose.y = previous.y + fraction * (p.y - previous.y);
      } else if (t < control_horizon_) {control_pose = p;}
      previous_time = t;
    }
    // Long constant-twist rollouts need a soft penalty: near the goal a short
    // curved tail otherwise makes stopping cheaper than any useful progress.
    // The executed prefix retains the strict corridor check above.
    return std::hypot(control_pose.x - target_.x, control_pose.y - target_.y) +
      0.5 * sum / trajectory.poses.size() + 0.25 * peak;
  }

private:
  struct Projection
  {
    geometry_msgs::msg::Pose2D point;
    double distance = std::numeric_limits<double>::infinity();
    size_t segment = 0;
  };
  Projection project(const geometry_msgs::msg::Pose2D & p) const
  {
    Projection best;
    if (points_.size() == 1) {
      return {points_.front(), std::hypot(p.x - points_.front().x, p.y - points_.front().y), 0};
    }
    double best_squared = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const auto & a = points_[i];
      const auto & segment = segments_[i];
      const double fraction = std::max(0.0, std::min(1.0,
        ((p.x - a.x) * segment.dx + (p.y - a.y) * segment.dy) * segment.inverse_squared));
      geometry_msgs::msg::Pose2D q;
      q.x = a.x + fraction * segment.dx; q.y = a.y + fraction * segment.dy;
      const double dx = p.x - q.x, dy = p.y - q.y;
      const double squared = dx * dx + dy * dy;
      if (squared < best_squared) {best_squared = squared; best.point = q; best.segment = i;}
    }
    best.distance = std::sqrt(best_squared);
    return best;
  }
  std::vector<geometry_msgs::msg::Pose2D> points_;
  struct Segment {double dx, dy, inverse_squared;};
  std::vector<Segment> segments_;
  geometry_msgs::msg::Pose2D target_;
  double lookahead_ = 0.65, control_horizon_ = 0.6, corridor_ = 0.12;
  double initial_distance_ = 0;
  double chord_limit_ = 0;
};
}  // namespace bot_navigation
PLUGINLIB_EXPORT_CLASS(bot_navigation::PathFollowCritic, dwb_core::TrajectoryCritic)
