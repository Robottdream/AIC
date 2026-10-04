// Copyright 2026 AIC contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace bot_navigation
{
struct TimedPoint {double x, y, t;};
struct Forecast {int id; double x, y, vx, vy, radius; std::vector<TimedPoint> samples;};
struct ForecastScore {bool collision = false; double cost = 0.0; double clearance = 1e6;};

inline TimedPoint forecastPosition(const Forecast & track, double t)
{
  if (track.samples.empty()) {return {track.x+track.vx*t, track.y+track.vy*t, t};}
  auto upper = std::lower_bound(track.samples.begin(), track.samples.end(), t,
    [](const TimedPoint & point, double value) {return point.t < value;});
  if (upper == track.samples.begin()) {return *upper;}
  if (upper == track.samples.end()) {return track.samples.back();}
  const auto & before = *(upper-1);
  const double f = (t-before.t)/(upper->t-before.t);
  return {before.x+(upper->x-before.x)*f, before.y+(upper->y-before.y)*f, t};
}

// Evaluate both actors at the same future time. Interpolation prevents a fast
// crossing between two DWB poses from escaping collision checks.
inline ForecastScore scoreForecast(
  const std::vector<TimedPoint> & path, const std::vector<Forecast> & tracks,
  double age, double robot_radius, double margin, double influence)
{
  ForecastScore result;
  double sum = 0.0, peak = 0.0;
  int count = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    const auto & end = path[i];
    const auto & begin = path[i ? i - 1 : i];
    const int samples = std::max(1, static_cast<int>(std::ceil((end.t - begin.t) / 0.05)));
    for (int j = 1; j <= samples; ++j) {
      const double f = static_cast<double>(j) / samples;
      const double t = begin.t + (end.t - begin.t) * f;
      const double x = begin.x + (end.x - begin.x) * f;
      const double y = begin.y + (end.y - begin.y) * f;
      double closest = 1e6;
      for (const auto & track : tracks) {
        const double elapsed = age + t;
        const auto obstacle = forecastPosition(track, elapsed);
        const double distance = std::hypot(x - obstacle.x, y - obstacle.y);
        // Measured surface enclosure plus increasing constant-velocity uncertainty.
        const double clearance = distance - track.radius - robot_radius - margin - 0.04 * elapsed;
        closest = std::min(closest, clearance);
      }
      result.clearance = std::min(result.clearance, closest);
      if (closest <= 0.0) {result.collision = true; return result;}
      const double penalty = std::exp(-closest / influence);
      peak = std::max(peak, penalty);
      sum += penalty;
      ++count;
    }
  }
  if (count) {result.cost = 0.5 * peak + 0.5 * sum / count;}
  return result;
}
}  // namespace bot_navigation
