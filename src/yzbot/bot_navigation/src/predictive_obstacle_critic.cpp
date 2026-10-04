// Copyright 2026 AIC contributors. SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "prediction_math.hpp"

namespace bot_navigation
{
class PredictiveObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
    auto parameter = [&](const std::string & key, double value) {
        if (!node->has_parameter(prefix + key)) {node->declare_parameter(prefix + key, value);}
        return node->get_parameter(prefix + key).as_double();
      };
    margin_ = parameter("margin", 0.08);
    influence_ = parameter("influence_distance", 0.35);
    max_age_ = parameter("max_observation_age", 0.35);
    if (margin_ < 0 || influence_ <= 0 || max_age_ <= 0) {
      throw std::invalid_argument("Invalid predictive critic distances/age");
    }
    for (const auto & point : costmap_ros_->getRobotFootprint()) {
      radius_ = std::max(radius_, std::hypot(point.x, point.y));
    }
    // Regular rclcpp publisher: diagnostics also work during lifecycle recovery.
    status_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node, "/navigation/predictive_planner", rclcpp::QoS(10));
    subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/navigation/predicted_obstacles", rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::vector<Forecast> parsed;
        std::string frame;
        double stamp = 0;
        bool enabled = false;
        std::string error;
        try {
          const auto data = nlohmann::json::parse(message->data);
          if (data.at("version").get<int>() != 1) {throw std::runtime_error("Unknown forecast version");}
          enabled = data.at("enabled").get<bool>();
          frame = data.at("frame").get<std::string>();
          stamp = data.at("stamp").get<double>();
          if (!std::isfinite(stamp)) {throw std::runtime_error("Nonfinite forecast stamp");}
          if (enabled) {
            for (const auto & item : data.at("tracks")) {
              Forecast t{item.at("id").get<int>(), item.at("center").at(0).get<double>(),
                item.at("center").at(1).get<double>(), item.at("velocity").at(0).get<double>(),
                item.at("velocity").at(1).get<double>(), item.at("radius").get<double>(), {}};
              if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.vx) ||
                !std::isfinite(t.vy) || !std::isfinite(t.radius) || t.radius <= 0 ||
                t.radius > 1.5 || std::hypot(t.vx, t.vy) > 1.2)
              {throw std::runtime_error("Invalid moving surface forecast");}
              if (item.contains("samples")) {
                double last = -1;
                for (const auto & sample : item.at("samples")) {
                  TimedPoint p{sample.at("center").at(0).get<double>(),
                    sample.at("center").at(1).get<double>(), sample.at("t").get<double>()};
                  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.t) ||
                    p.t < 0 || p.t <= last) {throw std::runtime_error("Invalid route sample");}
                  t.samples.push_back(p); last = p.t;
                }
                if (t.samples.empty() || t.samples.front().t != 0 || t.samples.back().t < 2.2) {
                  throw std::runtime_error("Route forecast does not cover the local horizon");
                }
              }
              parsed.push_back(t);
            }
          }
        } catch (const std::exception & e) {parsed.clear(); enabled = false; error = e.what();}
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(parsed);
        frame_ = frame; stamp_ = stamp; enabled_ = enabled; error_ = error;
        received_ = std::chrono::steady_clock::now();
      });
    RCLCPP_INFO(node->get_logger(), "PredictiveObstacle: timed lidar forecasts, radius %.3f m", radius_);
  }

  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_.clear(); checked_ = rejected_ = 0;
    age_ = node_.lock()->now().seconds() - stamp_;
    const double wall_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_).count();
    fresh_ = enabled_ && frame_ == costmap_ros_->getGlobalFrameID() &&
      age_ >= -0.1 && age_ <= max_age_ && wall_age < 0.6;
    if (fresh_) {active_ = latest_; age_ = std::max(0.0, age_);}
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    ++checked_; ++total_checked_;
    if (active_.empty()) {return 0;}
    const auto & offsets = trajectory.time_offsets;
    // Humble StandardTrajectoryGenerator adds a start pose and repeats the end
    // pose, but publishes one fewer offset. Its offsets are [0, dt, ..., horizon].
    // Pose i is at offset i; the repeated endpoint shares the final offset.
    const bool humble_endpoint = trajectory.poses.size() == offsets.size() + 1 &&
      offsets.size() >= 2 &&
      std::hypot(trajectory.poses.back().x - trajectory.poses[trajectory.poses.size()-2].x,
      trajectory.poses.back().y - trajectory.poses[trajectory.poses.size()-2].y) < 1e-6;
    if (trajectory.poses.size() != offsets.size() && !humble_endpoint) {
      throw dwb_core::IllegalTrajectoryException(name_, "Missing trajectory timestamps");
    }
    std::vector<TimedPoint> path;
    double previous = -1;
    for (size_t i = 0; i < trajectory.poses.size(); ++i) {
      const double t = rclcpp::Duration(offsets[std::min(i, offsets.size()-1)]).seconds();
      if (t < previous || !std::isfinite(t)) {
        throw dwb_core::IllegalTrajectoryException(name_, "Invalid trajectory timestamp order");
      }
      previous = t;
      path.push_back({trajectory.poses[i].x, trajectory.poses[i].y, t});
    }
    const auto result = scoreForecast(path, active_, age_, radius_, margin_, influence_);
    if (result.collision) {
      ++rejected_; ++total_rejected_;
      throw dwb_core::IllegalTrajectoryException(name_, "Predicted moving obstacle collision");
    }
    return result.cost;
  }

  void debrief(const nav_2d_msgs::msg::Twist2D & chosen) override
  {
    const double now = node_.lock()->now().seconds();
    if (now >= last_status_ && now - last_status_ < 0.25) {return;}
    last_status_ = now;
    std::string error;
    {std::lock_guard<std::mutex> lock(mutex_); error = error_;}
    nlohmann::json data{{"stamp", now}, {"fresh", fresh_}, {"active_tracks", active_.size()},
      {"observation_age", age_}, {"candidates", checked_}, {"rejected", rejected_},
      {"total_candidates", total_checked_}, {"total_rejected", total_rejected_},
      {"chosen", {chosen.x, chosen.y, chosen.theta}}, {"error", error}};
    std_msgs::msg::String message; message.data = data.dump(); status_->publish(message);
  }

private:
  std::mutex mutex_;
  std::vector<Forecast> latest_, active_;
  std::string frame_, error_;
  std::chrono::steady_clock::time_point received_{};
  double stamp_ = 0, age_ = 0, last_status_ = -1;
  double margin_ = 0.08, radius_ = 0, influence_ = 0.35, max_age_ = 0.35;
  bool enabled_ = false, fresh_ = false;
  size_t checked_ = 0, rejected_ = 0, total_checked_ = 0, total_rejected_ = 0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
}  // namespace bot_navigation
PLUGINLIB_EXPORT_CLASS(bot_navigation::PredictiveObstacleCritic, dwb_core::TrajectoryCritic)
