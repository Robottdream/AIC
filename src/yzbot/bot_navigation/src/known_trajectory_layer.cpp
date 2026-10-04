// Copyright 2026 AIC contributors. SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <mutex>
#include <nlohmann/json.hpp>
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"

namespace bot_navigation
{
class KnownTrajectoryLayer : public nav2_costmap_2d::Layer
{
  struct Sample {double x, y, radius, t;};
  std::mutex mutex_;
  std::vector<Sample> latest_, active_, previous_;
  double stamp_ = 0, robot_radius_ = 0;
  std::string frame_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

public:
  void onInitialize() override
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".enabled", enabled_);
    for (const auto & p : getFootprint()) {
      robot_radius_ = std::max(robot_radius_, std::hypot(p.x, p.y));
    }
    current_ = true;
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/navigation/known_scene_predictions", rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::vector<Sample> samples;
        std::string frame;
        double stamp = 0;
        try {
          const auto data = nlohmann::json::parse(message->data);
          frame = data.at("frame").get<std::string>();
          stamp = data.at("stamp").get<double>();
          if (data.at("version").get<int>() != 1 || !std::isfinite(stamp)) {
            throw std::runtime_error("Invalid known route forecast header");
          }
          if (data.at("enabled").get<bool>()) {
            for (const auto & track : data.at("tracks")) {
              const double radius = track.at("radius").get<double>();
              if (!std::isfinite(radius) || radius <= 0 || radius > 1.5) {
                throw std::runtime_error("Invalid known obstacle radius");
              }
              int index = 0;
              for (const auto & p : track.at("samples")) {
                Sample s{p.at("center").at(0).get<double>(),
                  p.at("center").at(1).get<double>(), radius, p.at("t").get<double>()};
                if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.t)) {
                  throw std::runtime_error("Nonfinite known obstacle sample");
                }
                if (s.t >= 0 && s.t <= 4.0 && index++ % 2 == 0) {samples.push_back(s);}
              }
            }
          }
        } catch (const std::exception &) {samples.clear();}
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(samples); frame_ = frame; stamp_ = stamp;
        received_ = std::chrono::steady_clock::now();
      }, options);
    RCLCPP_INFO(logger_, "KnownTrajectoryLayer: 4s scene route preview in global planning costs");
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.clear(); current_ = true;
    // Keep previous bounds until the next update clears the old preview.
  }
  bool isClearable() override {return false;}
  void onFootprintChanged() override
  {
    robot_radius_ = 0;
    for (const auto & p : getFootprint()) {
      robot_radius_ = std::max(robot_radius_, std::hypot(p.x, p.y));
    }
  }

  void updateBounds(double, double, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    active_.clear();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double age = clock_->now().seconds() - stamp_;
      const double wall_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - received_).count();
      if (enabled_ && frame_ == layered_costmap_->getGlobalFrameID() &&
        age >= -0.1 && age < .4 && wall_age < .6) {active_ = latest_;}
    }
    auto include = [&](const std::vector<Sample> & samples) {
        for (const auto & p : samples) {
          const double r = p.radius + robot_radius_ + .08;
          *min_x = std::min(*min_x, p.x-r); *max_x = std::max(*max_x, p.x+r);
          *min_y = std::min(*min_y, p.y-r); *max_y = std::max(*max_y, p.y+r);
        }
      };
    include(previous_); include(active_); previous_ = active_;
    current_ = true;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    for (const auto & p : active_) {
      const double radius = p.radius + robot_radius_ + .08;
      int left, bottom, right, top;
      grid.worldToMapNoBounds(p.x-radius, p.y-radius, left, bottom);
      grid.worldToMapNoBounds(p.x+radius, p.y+radius, right, top);
      left = std::max({left, min_i, 0}); bottom = std::max({bottom, min_j, 0});
      right = std::min({right+1, max_i, static_cast<int>(grid.getSizeInCellsX())});
      top = std::min({top+1, max_j, static_cast<int>(grid.getSizeInCellsY())});
      for (int y = bottom; y < top; ++y) {
        for (int x = left; x < right; ++x) {
          double wx, wy; grid.mapToWorld(x, y, wx, wy);
          const double distance = std::hypot(wx-p.x, wy-p.y);
          if (distance > radius) {continue;}
          // Soft preview: future routes are costly, not permanent solid walls.
          // Actual obstacles keep their original lethal/unknown costs.
          const auto penalty = static_cast<unsigned char>(
            (240.0 - 150.0*distance/radius) * (1.0 - p.t/8.0));
          const auto old = grid.getCost(x, y);
          if (old < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            grid.setCost(x, y, std::max(old, penalty));
          }
        }
      }
    }
  }
};
}  // namespace bot_navigation
PLUGINLIB_EXPORT_CLASS(bot_navigation::KnownTrajectoryLayer, nav2_costmap_2d::Layer)
