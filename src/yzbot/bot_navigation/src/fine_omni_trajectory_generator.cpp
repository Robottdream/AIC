#include <cmath>
#include <vector>
#include "dwb_plugins/standard_traj_generator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bot_navigation
{
// Retain the normal full-speed search and its acceleration/braking simulation.
// Add a small grid near zero instead of multiplying the whole velocity grid.
class FineOmniTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & velocity) override
  {
    dwb_plugins::StandardTrajectoryGenerator::startNewIteration(velocity);
    fine_.clear();
    index_ = 0;
    auto limits = kinematics_handler_->getKinematics();
    for (int x = -3; x <= 3; ++x) {
      for (int y = -3; y <= 3; ++y) {
        nav_2d_msgs::msg::Twist2D cmd;
        cmd.x = x * 0.025;
        cmd.y = y * 0.025;
        cmd.theta = 0.0;
        const double speed = std::hypot(cmd.x, cmd.y);
        if ((x == 0 && y == 0) || cmd.x < limits.getMinX() || cmd.x > limits.getMaxX() ||
          cmd.y < limits.getMinY() || cmd.y > limits.getMaxY() ||
          speed < limits.getMinSpeedXY() || speed > limits.getMaxSpeedXY() ||
          limits.getMinSpeedTheta() > 0.0)
        {continue;}
        fine_.push_back(cmd);
      }
    }
    // Retain 0.2 m/s maneuvering resolution when the full-speed grid becomes
    // coarser at 2--4 m/s. Avoid increasing the entire velocity grid quadratically.
    if (limits.getMaxSpeedXY() > 1.0 + 1e-6) {
      for (int x = -5; x <= 5; ++x) {
        for (int y = -5; y <= 5; ++y) {
          nav_2d_msgs::msg::Twist2D cmd;
          cmd.x = x * 0.2; cmd.y = y * 0.2; cmd.theta = 0;
          const double speed = std::hypot(cmd.x, cmd.y);
          if (speed < 1e-6 || speed > 1.0 + 1e-6 ||
            cmd.x < limits.getMinX() || cmd.x > limits.getMaxX() ||
            cmd.y < limits.getMinY() || cmd.y > limits.getMaxY() ||
            speed < limits.getMinSpeedXY() || limits.getMinSpeedTheta() > 0) {continue;}
          fine_.push_back(cmd);
        }
      }
    }
  }

  bool hasMoreTwists() override
  {
    return index_ < fine_.size() || dwb_plugins::StandardTrajectoryGenerator::hasMoreTwists();
  }

  nav_2d_msgs::msg::Twist2D nextTwist() override
  {
    if (index_ < fine_.size()) {return fine_[index_++];}
    return dwb_plugins::StandardTrajectoryGenerator::nextTwist();
  }

private:
  std::vector<nav_2d_msgs::msg::Twist2D> fine_;
  size_t index_{0};
};
}  // namespace bot_navigation

PLUGINLIB_EXPORT_CLASS(bot_navigation::FineOmniTrajectoryGenerator, dwb_core::TrajectoryGenerator)
