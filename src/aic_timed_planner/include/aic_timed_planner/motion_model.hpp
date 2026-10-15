#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace aic_timed_planner
{
constexpr double PI = 3.14159265358979323846;
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
struct Motion {double sx, sy, ex, ey, size, speed;};
struct Observation {double stamp=0, along=0, direction=1, gain=.965; bool valid=false;};
struct Point {double x, y;};
// A route preference only: neither speed limiting nor a new lethal boundary.
inline double clearance_cost(double clearance,double preferred,double weight) {
  const double deficit=std::max(0.,1.-clearance/std::max(.01,preferred));
  return std::max(0.,weight)*deficit*deficit;
}
struct Pose2 {double x, y, yaw;};
struct FrameAlignment {
  Pose2 map, world;
  Pose2 toWorld(Pose2 p) const {
    const double a=wrap(world.yaw-map.yaw),c=std::cos(a),s=std::sin(a);
    return {world.x+(p.x-map.x)*c-(p.y-map.y)*s,
            world.y+(p.x-map.x)*s+(p.y-map.y)*c,wrap(p.yaw+a)};
  }
};
struct KinematicState {double x,y,yaw,v,w;};
inline void advance(KinematicState & state,double target_v,double target_w,double dt) {
  state.v+=std::clamp(target_v-state.v,-5.*dt,3.5*dt);
  state.w+=std::clamp(target_w-state.w,-10.*dt,10.*dt);
  const double middle=state.yaw+state.w*dt/2.;
  state.x+=state.v*std::cos(middle)*dt;state.y+=state.v*std::sin(middle)*dt;
  state.yaw=wrap(state.yaw+state.w*dt);
}

inline std::vector<Point> forecast(const Motion & m, Observation state, double now, double horizon)
{
  const double length=std::hypot(m.ex-m.sx,m.ey-m.sy);
  const double ux=(m.ex-m.sx)/length, uy=(m.ey-m.sy)/length;
  double along=std::clamp(state.along,0.,length), direction=state.direction;
  const double advance=std::max(0.,now-state.stamp);
  std::vector<Point> result;
  double t=-advance;
  while(t<horizon+.025) {
    const double remaining=direction>0 ? length-along : along;
    if(remaining<.20) direction=-direction;
    const double distance=direction>0 ? length-along : along;
    if(t>=-.001) result.push_back({m.sx+along*ux,m.sy+along*uy});
    along+=direction*state.gain*std::min(m.speed,distance)*.025;
    along=std::clamp(along,0.,length); t+=.025;
  }
  if(result.empty()) result.push_back({m.sx+along*ux,m.sy+along*uy});
  return result;
}
inline bool overlaps(double x, double y, double yaw, double hx, double hy,
                     Point obstacle, double half)
{
  const double c=std::cos(yaw),s=std::sin(yaw),dx=obstacle.x-x,dy=obstacle.y-y;
  return std::abs(dx)<=half+hx*std::abs(c)+hy*std::abs(s) &&
    std::abs(dy)<=half+hx*std::abs(s)+hy*std::abs(c) &&
    std::abs(dx*c+dy*s)<=hx+half*(std::abs(c)+std::abs(s)) &&
    std::abs(-dx*s+dy*c)<=hy+half*(std::abs(c)+std::abs(s));
}
// Soft preference against nodes an approaching actor will soon sweep if
// tracking temporarily slows there. This is a rolling forecast, not a track mask.
inline double future_occupancy_risk(double x,double y,double yaw,double hx,double hy,
                                   const std::vector<Point> & future,double at,double half) {
  if(future.empty()) return 0.;
  double risk=0;
  for(double lead:{1.,2.,3.,4.}) {
    const auto i=std::min(future.size()-1,size_t(std::max(0.,at+lead)/.025));
    if(overlaps(x,y,yaw,hx,hy,future[i],half)) risk+=.25;
  }
  return risk;
}
inline double turn_time(double radians)
{
  const double a=std::abs(wrap(radians));
  return a<=1.6 ? 2*std::sqrt(a/10.) : a/4.+.4;
}
}  // namespace aic_timed_planner
