#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>
#include "aic_timed_planner/motion_model.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace aic_timed_planner
{
class TimeAwarePlanner : public nav2_core::GlobalPlanner
{
  struct Grid {
    std::shared_ptr<nav2_costmap_2d::Costmap2D> map;
    std::vector<float> clearance;
    unsigned w=0,h=0;
    double r=0,ox=0,oy=0;
  };
  struct SearchNode {
    double x,y,yaw,t,v,g; int heading,parent;
  };
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_,frame_;
  std::mutex data_mutex_;
  std::shared_ptr<Grid> grid_;
  std::vector<Motion> motions_;
  std::vector<Observation> observations_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> motion_subs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  std::unordered_map<unsigned,std::vector<float>> heuristic_cache_;
  const Grid * heuristic_grid_=nullptr;
  double speed_=2.2,accel_=3.5,decel_=5.,limit_=1.5,horizon_=90.;
  double robot_speed_=0.,odom_stamp_=0.;
  bool slow_near_static_obstacles_=true;
  const double step_=.20;
  const int headings_=32;

  static void field(const std::shared_ptr<Grid> & g, std::vector<float> & values,
                    const std::vector<unsigned> & seeds, bool allow_near_wall,
                    bool slow_near_static_obstacles=true, double nominal_speed=2.2)
  {
    using Item=std::pair<float,unsigned>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    values.assign(g->w*g->h,1e9f);
    for(auto index:seeds) {values[index]=0;queue.push({0,index});}
    const int offsets[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
    while(!queue.empty()) {
      auto [distance,index]=queue.top();queue.pop();
      if(distance>values[index]+1e-6) continue;
      const int x=index%g->w,y=index/g->w;
      for(auto & offset:offsets) {
        const int nx=x+offset[0],ny=y+offset[1];
        if(nx<0||ny<0||nx>=int(g->w)||ny>=int(g->h)) continue;
        const unsigned next=ny*g->w+nx;
        if(!allow_near_wall && g->clearance[next]<.30) continue;
        const double cruise=allow_near_wall ? 1. :
          ((slow_near_static_obstacles && (g->clearance[next]<1.10 || g->clearance[index]<1.10)) ?
           std::min(1.2,nominal_speed) : nominal_speed);
        const float candidate=distance+g->r*(offset[0]&&offset[1] ? 1.41421356237 : 1.)/cruise;
        if(candidate+1e-6<values[next]) {values[next]=candidate;queue.push({candidate,next});}
      }
    }
  }

  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    if(msg->info.resolution<=0 || std::abs(tf2::getYaw(msg->info.origin.orientation))>1e-6) {
      RCLCPP_ERROR(node_->get_logger(),"Timed planner requires a valid world-aligned static map");return;
    }
    auto g=std::make_shared<Grid>();
    g->w=msg->info.width;g->h=msg->info.height;g->r=msg->info.resolution;
    g->ox=msg->info.origin.position.x;g->oy=msg->info.origin.position.y;
    if(msg->data.size()!=g->w*g->h) return;
    g->map=std::make_shared<nav2_costmap_2d::Costmap2D>(g->w,g->h,g->r,g->ox,g->oy,0);
    std::vector<unsigned> occupied;
    for(unsigned i=0;i<msg->data.size();++i) {
      if(msg->data[i]<0 || msg->data[i]>=65) {
        g->map->setCost(i%g->w,i/g->w,254);occupied.push_back(i);
      }
    }
    field(g,g->clearance,occupied,true);
    std::lock_guard<std::mutex> lock(data_mutex_);grid_=g;
  }

  void motionCallback(size_t index,std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
  {
    if(msg->data.size()!=4) return;
    for(double value:msg->data) if(!std::isfinite(value)) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto & old=observations_[index]; const auto & m=motions_[index];
    const double length=std::hypot(m.ex-m.sx,m.ey-m.sy);
    const double along=((msg->data[1]-m.sx)*(m.ex-m.sx)+(msg->data[2]-m.sy)*(m.ey-m.sy))/length;
    const double dt=msg->data[0]-old.stamp;
    double gain=old.valid ? old.gain : .965;
    const double direction=msg->data[3]>0 ? 1. : -1.;
    const double remaining=direction>0 ? length-along : along;
    if(old.valid && dt>.02 && dt<.3 && old.direction==direction && remaining>.40) {
      const double observed=std::abs(along-old.along)/(m.speed*dt);
      gain=.9*gain+.1*std::clamp(observed,.80,1.04);
    }
    old={msg->data[0],std::clamp(along,0.,length),direction,gain,true};
  }

  std::vector<float> heuristic(const std::shared_ptr<Grid> & g,unsigned goal)
  {
    // Cache only within an unchanged map. The map pointer identifies versions.
    if(heuristic_grid_!=g.get()) {heuristic_cache_.clear();heuristic_grid_=g.get();}
    auto found=heuristic_cache_.find(goal);
    if(found!=heuristic_cache_.end()) return found->second;
    std::vector<float> result;field(g,result,{goal},false,slow_near_static_obstacles_,speed_);
    if(heuristic_cache_.size()>24) heuristic_cache_.clear();
    heuristic_cache_[goal]=result;return result;
  }

public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,std::shared_ptr<tf2_ros::Buffer>,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_=parent.lock();name_=name;costmap_ros_=costmap_ros;frame_=costmap_ros_->getGlobalFrameID();
    auto declare=[&](const std::string & key,const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node_,name_+"."+key,value);
    };
    declare("obstacle_names",rclcpp::ParameterValue(std::vector<std::string>{}));
    declare("obstacle_tracks",rclcpp::ParameterValue(std::vector<double>{}));
    declare("obstacle_sizes",rclcpp::ParameterValue(std::vector<double>{}));
    declare("obstacle_speeds",rclcpp::ParameterValue(std::vector<double>{}));
    declare("nominal_speed",rclcpp::ParameterValue(2.2));
    declare("slow_near_static_obstacles",rclcpp::ParameterValue(true));
    declare("max_planning_time",rclcpp::ParameterValue(1.5));
    node_->get_parameter(name_+".nominal_speed",speed_);
    node_->get_parameter(name_+".slow_near_static_obstacles",slow_near_static_obstacles_);
    node_->get_parameter(name_+".max_planning_time",limit_);
    std::vector<std::string> names;std::vector<double> tracks,sizes,speeds;
    node_->get_parameter(name_+".obstacle_names",names);
    node_->get_parameter(name_+".obstacle_tracks",tracks);
    node_->get_parameter(name_+".obstacle_sizes",sizes);
    node_->get_parameter(name_+".obstacle_speeds",speeds);
    if(tracks.size()!=names.size()*4 || sizes.size()!=names.size() || speeds.size()!=names.size())
      throw std::runtime_error("Known motion dimensions disagree");
    observations_.resize(names.size());
    for(size_t i=0;i<names.size();++i) {
      Motion m{tracks[4*i],tracks[4*i+1],tracks[4*i+2],tracks[4*i+3],sizes[i],speeds[i]};
      if(std::hypot(m.ex-m.sx,m.ey-m.sy)<.4 || m.size<=0 || m.speed<=0)
        throw std::runtime_error("Invalid known obstacle motion");
      motions_.push_back(m);
      motion_subs_.push_back(node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/"+names[i]+"/motion_state",rclcpp::SensorDataQoS(),
        [this,i](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){motionCallback(i,msg);}));
    }
    map_sub_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&TimeAwarePlanner::mapCallback,this,std::placeholders::_1));
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_speed_=std::hypot(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
        odom_stamp_=rclcpp::Time(msg->header.stamp).seconds();
      });
    debug_pub_=node_->create_publisher<std_msgs::msg::String>("/known_timed_plan",10);
    RCLCPP_INFO(node_->get_logger(),"Known-motion planner: continuous SE2/time search, no waiting actions");
  }
  void activate() override {debug_pub_->on_activate();}
  void deactivate() override {debug_pub_->on_deactivate();}
  void cleanup() override {motion_subs_.clear();map_sub_.reset();odom_sub_.reset();grid_.reset();heuristic_cache_.clear();}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                const geometry_msgs::msg::PoseStamped & goal) override
  {
    const auto began=std::chrono::steady_clock::now();
    const double now=node_->now().seconds();
    nav_msgs::msg::Path result;result.header.frame_id=frame_;result.header.stamp=node_->now();
    std::shared_ptr<Grid> grid;std::vector<Observation> observed;double current_speed;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      grid=grid_;observed=observations_;
      current_speed=now-odom_stamp_<.5 ? robot_speed_ : 0.;
    }
    if(!grid || start.header.frame_id!=frame_ || goal.header.frame_id!=frame_) return result;
    unsigned gx,gy,sx,sy;
    if(!grid->map->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy) ||
       !grid->map->worldToMap(start.pose.position.x,start.pose.position.y,sx,sy)) return result;
    const auto guide=heuristic(grid,gy*grid->w+gx);
    if(guide[sy*grid->w+sx]>=1e8) return result;
    geometry_msgs::msg::PoseStamped robot;
    const bool virtual_start=costmap_ros_->getRobotPose(robot) &&
      std::hypot(robot.pose.position.x-start.pose.position.x,robot.pose.position.y-start.pose.position.y)>.75;
    const bool dynamic=!virtual_start && !motions_.empty();
    std::vector<std::vector<Point>> futures;
    if(dynamic) {
      for(size_t i=0;i<observed.size();++i) {
        if(!observed[i].valid || now-observed[i].stamp>.35 || observed[i].stamp-now>.15) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,"Known obstacle state unavailable or stale");return result;
        }
        futures.push_back(forecast(motions_[i],observed[i],now,horizon_+2));
      }
    }
    auto footprint=costmap_ros_->getRobotFootprint();
    double hx=0,hy=0;
    for(auto p:footprint) {hx=std::max(hx,std::abs(p.x));hy=std::max(hy,std::abs(p.y));}
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(grid->map.get());
    auto static_valid=[&](double x,double y,double yaw) {
      unsigned mx,my;if(!grid->map->worldToMap(x,y,mx,my)) return false;
      if(grid->map->getCost(mx,my)>=253) return false;
      return checker.footprintCostAtPose(x,y,yaw,footprint)<253;
    };
    bool searching=true;
    auto valid=[&](double x,double y,double yaw,double t) {
      if(!static_valid(x,y,yaw)) return false;
      if(!dynamic) return true;
      const double uncertainty=std::min(1.5,.40+.035*t);
      for(size_t i=0;i<futures.size();++i) {
        // Check a continuous occupancy interval, not only one instant.
        for(double offset=-uncertainty;offset<=uncertainty+(searching ? .30 : .025);offset+=.10) {
          const double when=std::max(0.,t+offset);
          const size_t index=std::min(futures[i].size()-1,size_t(when/.025));
          if(overlaps(x,y,yaw,hx,hy,futures[i][index],motions_[i].size/2+(searching ? .18 : .12))) return false;
        }
      }
      return true;
    };
    const double start_yaw=tf2::getYaw(start.pose.orientation),goal_yaw=tf2::getYaw(goal.pose.orientation);
    if(!valid(start.pose.position.x,start.pose.position.y,start_yaw,0)) return result;
    using Item=std::pair<double,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
    std::vector<SearchNode> nodes;nodes.reserve(120000);
    std::unordered_map<uint64_t,double> best;best.reserve(120000);
    auto key=[&](double x,double y,int heading,double t) {
      const int xx=std::max(0,int(std::round((x-grid->ox)/.10)));
      const int yy=std::max(0,int(std::round((y-grid->oy)/.10)));
      const int tt=dynamic ? int(t/.25) : 0;
      return (uint64_t(tt)<<29)|(uint64_t(heading)<<24)|(uint64_t(yy)<<12)|uint64_t(xx);
    };
    auto estimate=[&](double x,double y) {
      unsigned mx,my;if(!grid->map->worldToMap(x,y,mx,my)) return 1e9;
      return double(guide[my*grid->w+mx]);
    };
    const int nearest=(int(std::round(start_yaw*headings_/(2*PI)))%headings_+headings_)%headings_;
    for(int h=0;h<headings_;++h) {
      if(!virtual_start && current_speed>.10 && h!=nearest) continue;
      const double yaw=h*2*PI/headings_,rotation=wrap(yaw-start_yaw),dt=turn_time(rotation);
      bool clear=true;
      for(double f=0;f<=1.001;f+=.10) {
        if(!valid(start.pose.position.x,start.pose.position.y,start_yaw+rotation*f,dt*f)) {clear=false;break;}
      }
      if(!clear) continue;
      SearchNode node{start.pose.position.x,start.pose.position.y,yaw,dt,
                      h==nearest ? current_speed : 0.,dt,h,-1};
      const int index=nodes.size();nodes.push_back(node);best[key(node.x,node.y,h,dt)]=dt;
      open.push({dt+1.1*estimate(node.x,node.y),index});
    }
    int found=-1;unsigned iterations=0;
    while(!open.empty()) {
      const auto [priority,index]=open.top();(void)priority;open.pop();
      const SearchNode parent=nodes[index];
      if(parent.g>best[key(parent.x,parent.y,parent.heading,parent.t)]+1e-6) continue;
      if(++iterations%128==0 && std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>limit_) break;
      if(nodes.size()>600000) break;
      const double distance=std::hypot(parent.x-goal.pose.position.x,parent.y-goal.pose.position.y);
      if(distance<.13) {
        const double delta=wrap(goal_yaw-parent.yaw),dt=turn_time(delta);
        bool clear=true;
        for(double f=0;f<=1.001;f+=.10) {
          const double x=parent.x+f*(goal.pose.position.x-parent.x),y=parent.y+f*(goal.pose.position.y-parent.y);
          if(!valid(x,y,parent.yaw+delta*f,parent.t+dt*f)) {clear=false;break;}
        }
        if(clear) {found=index;break;}
      }
      for(int turn=-2;turn<=2;++turn) {
        const int heading=(parent.heading+turn+headings_)%headings_;
        const double delta=turn*2*PI/headings_,yaw=parent.yaw+delta;
        const double radius=turn ? step_/delta : 0;
        const double x=turn ? parent.x+radius*(std::sin(yaw)-std::sin(parent.yaw)) : parent.x+step_*std::cos(yaw);
        const double y=turn ? parent.y-radius*(std::cos(yaw)-std::cos(parent.yaw)) : parent.y+step_*std::sin(yaw);
        unsigned mx,my;if(!grid->map->worldToMap(x,y,mx,my)) continue;
        const double clearance=grid->clearance[my*grid->w+mx];
        double velocity=slow_near_static_obstacles_ && clearance<1.10 ? std::min(1.2,speed_) : speed_;
        if(turn) velocity=std::max(.18,velocity*std::min(1.,std::abs(radius)/1.1));
        if(velocity>parent.v) velocity=std::min(velocity,std::sqrt(parent.v*parent.v+2*accel_*step_));
        else velocity=std::max(velocity,std::sqrt(std::max(0.,parent.v*parent.v-2*decel_*step_)));
        const double dt=2*step_/std::max(.18,velocity+parent.v),arrival=parent.t+dt;
        if(arrival>horizon_) continue;
        bool clear=true;
        for(double f=.25;f<=1.001;f+=.25) {
          const double angle=parent.yaw+delta*f;
          const double xx=turn ? parent.x+radius*(std::sin(angle)-std::sin(parent.yaw)) : parent.x+step_*f*std::cos(yaw);
          const double yy=turn ? parent.y-radius*(std::cos(angle)-std::cos(parent.yaw)) : parent.y+step_*f*std::sin(yaw);
          if(!valid(xx,yy,angle,parent.t+dt*f)) {clear=false;break;}
        }
        if(!clear) continue;
        const double cost=parent.g+dt+.06*std::abs(delta)+.03*dt*std::max(0.,1.-clearance);
        const auto hash=key(x,y,heading,arrival);
        auto previous=best.find(hash);if(previous!=best.end()&&previous->second<=cost) continue;
        best[hash]=cost;
        const int next=nodes.size();nodes.push_back({x,y,yaw,arrival,velocity,cost,heading,index});
        open.push({cost+1.1*estimate(x,y),next});
      }
    }
    if(found<0) {
      RCLCPP_WARN(node_->get_logger(),"Timed route search did not find a continuous path in %.2fs (%zu nodes)",limit_,nodes.size());return result;
    }
    std::vector<int> chain;for(int index=found;index>=0;index=nodes[index].parent) chain.push_back(index);
    std::reverse(chain.begin(),chain.end());
    if(dynamic) {
      // Revalidate against the latest state after search latency, from the
      // measured progress on the route. Never publish an already stale plan.
      const double latest_now=node_->now().seconds();
      std::vector<Observation> latest;
      {std::lock_guard<std::mutex> lock(data_mutex_);latest=observations_;}
      searching=false;
      futures.clear();
      for(size_t i=0;i<latest.size();++i) {
        if(!latest[i].valid || latest_now-latest[i].stamp>.35) return result;
        futures.push_back(forecast(motions_[i],latest[i],latest_now,horizon_+2));
      }
      geometry_msgs::msg::PoseStamped measured;
      size_t closest=0;double minimum=1e9;
      if(costmap_ros_->getRobotPose(measured)) {
        for(size_t i=0;i<chain.size();++i) {
          auto n=nodes[chain[i]];
          const double d=std::hypot(measured.pose.position.x-n.x,measured.pose.position.y-n.y);
          if(d<minimum) {minimum=d;closest=i;}
        }
        if(minimum>.45) return result;
      }
      double offset=nodes[chain[closest]].t;
      if(closest==0) {
        // At a stationary start the initial alignment is still ahead, not
        // elapsed progress. Preserve its remaining time in the forecast.
        offset-=turn_time(nodes[chain[0]].yaw-tf2::getYaw(measured.pose.orientation));
      }
      for(size_t i=closest;i<chain.size();++i) {
        auto n=nodes[chain[i]];
        if(!valid(n.x,n.y,n.yaw,std::max(0.,n.t-offset))) {
          RCLCPP_WARN(node_->get_logger(),"Fresh-state validation rejected timed route at %.2fs",n.t-offset);
          return result;
        }
      }
    }
    result.poses.push_back(start);
    std::ostringstream times;times<<"[0";
    for(int index:chain) {
      const auto & n=nodes[index];geometry_msgs::msg::PoseStamped pose;
      pose.header=result.header;pose.pose.position.x=n.x;pose.pose.position.y=n.y;
      tf2::Quaternion q;q.setRPY(0,0,n.yaw);pose.pose.orientation=tf2::toMsg(q);
      result.poses.push_back(pose);times<<","<<n.t;
    }
    auto end=goal;end.header=result.header;result.poses.push_back(end);
    const auto & last=nodes[found];times<<","<<last.t+turn_time(goal_yaw-last.yaw)<<"]";
    if(dynamic && debug_pub_->is_activated()) {
      std_msgs::msg::String message;std::ostringstream json;
      json<<"{\"sim_start\":"<<now<<",\"dynamic\":true,\"goal\":["<<goal.pose.position.x<<","<<goal.pose.position.y
          <<"],\"arrival_seconds\":"<<times.str()<<",\"points\":[";
      for(size_t i=0;i<result.poses.size();++i) {
        if(i) json<<",";
        auto p=result.poses[i];
        json<<"["<<p.pose.position.x<<","<<p.pose.position.y<<","<<tf2::getYaw(p.pose.orientation)<<"]";
      }
      json<<"]}";message.data=json.str();debug_pub_->publish(message);
    }
    return result;
  }
};
}  // namespace aic_timed_planner
PLUGINLIB_EXPORT_CLASS(aic_timed_planner::TimeAwarePlanner,nav2_core::GlobalPlanner)
