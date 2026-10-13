#include <mutex>
#include <sstream>
#include "aic_timed_planner/motion_model.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"

namespace aic_timed_planner
{
// Keep RPP's geometric tracking and dynamic profiles. Replace its frozen-obstacle
// collision extrapolation with a receding-horizon check of the moving actors.
class TimeAwareController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  std::mutex data_mutex_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> static_map_;
  std::vector<Motion> motions_;
  std::vector<Observation> observations_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> motion_subs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr robot_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  double robot_x_=0,robot_y_=0,robot_yaw_=0,robot_stamp_=-1;
  double robot_v_=0,robot_w_=0;

public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    RegulatedPurePursuitController::configure(parent,name,tf,costmap);
    parent_=parent.lock();
    std::vector<std::string> names;std::vector<double> tracks,sizes,speeds;
    auto declare=[&](const std::string & key,const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(parent_,name+"."+key,value);
    };
    declare("obstacle_names",rclcpp::ParameterValue(std::vector<std::string>{}));
    declare("obstacle_tracks",rclcpp::ParameterValue(std::vector<double>{}));
    declare("obstacle_sizes",rclcpp::ParameterValue(std::vector<double>{}));
    declare("obstacle_speeds",rclcpp::ParameterValue(std::vector<double>{}));
    parent_->get_parameter(name+".obstacle_names",names);
    parent_->get_parameter(name+".obstacle_tracks",tracks);
    parent_->get_parameter(name+".obstacle_sizes",sizes);
    parent_->get_parameter(name+".obstacle_speeds",speeds);
    if(names.empty() || tracks.size()!=names.size()*4 || sizes.size()!=names.size() || speeds.size()!=names.size())
      throw std::runtime_error("Predictive controller requires known moving actors");
    observations_.resize(names.size());
    for(size_t i=0;i<names.size();++i) {
      motions_.push_back({tracks[4*i],tracks[4*i+1],tracks[4*i+2],tracks[4*i+3],sizes[i],speeds[i]});
      motion_subs_.push_back(parent_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/"+names[i]+"/motion_state",rclcpp::SensorDataQoS(),
        [this,i](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          if(msg->data.size()!=4) return;
          for(double d:msg->data) if(!std::isfinite(d)) return;
          const auto & m=motions_[i];double length=std::hypot(m.ex-m.sx,m.ey-m.sy);
          double along=((msg->data[1]-m.sx)*(m.ex-m.sx)+(msg->data[2]-m.sy)*(m.ey-m.sy))/length;
          std::lock_guard<std::mutex> lock(data_mutex_);
          observations_[i]={msg->data[0],std::clamp(along,0.,length),msg->data[3]>0 ? 1. : -1.,.965,true};
        }));
    }
    map_sub_=parent_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if(msg->info.resolution<=0 || msg->data.size()!=msg->info.width*msg->info.height ||
          std::abs(tf2::getYaw(msg->info.origin.orientation))>1e-6) return;
        auto map=std::make_shared<nav2_costmap_2d::Costmap2D>(msg->info.width,msg->info.height,
          msg->info.resolution,msg->info.origin.position.x,msg->info.origin.position.y,0);
        for(unsigned i=0;i<msg->data.size();++i)
          if(msg->data[i]<0 || msg->data[i]>=65) map->setCost(i%msg->info.width,i/msg->info.width,254);
        std::lock_guard<std::mutex> lock(data_mutex_);static_map_=map;
      });
    robot_sub_=parent_->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states",rclcpp::SensorDataQoS(),[this](gazebo_msgs::msg::ModelStates::ConstSharedPtr msg) {
        auto found=std::find(msg->name.begin(),msg->name.end(),"six_arm");
        if(found==msg->name.end()) return;
        auto i=std::distance(msg->name.begin(),found);
        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_x_=msg->pose[i].position.x;robot_y_=msg->pose[i].position.y;
        robot_yaw_=tf2::getYaw(msg->pose[i].orientation);robot_stamp_=parent_->now().seconds();
        robot_v_=msg->twist[i].linear.x*std::cos(robot_yaw_)+msg->twist[i].linear.y*std::sin(robot_yaw_);
        robot_w_=msg->twist[i].angular.z;
      });
    debug_pub_=parent_->create_publisher<std_msgs::msg::String>("/known_predictive_control",10);
  }
  void activate() override {RegulatedPurePursuitController::activate();debug_pub_->on_activate();}
  void deactivate() override {debug_pub_->on_deactivate();RegulatedPurePursuitController::deactivate();}
  void cleanup() override {
    motion_subs_.clear();map_sub_.reset();robot_sub_.reset();static_map_.reset();
    RegulatedPurePursuitController::cleanup();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    auto command=RegulatedPurePursuitController::computeVelocityCommands(pose,velocity,goal_checker);
    double now=parent_->now().seconds(),rx,ry,ra,rv,rw;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> map;
    std::vector<Observation> observations;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if(!static_map_ || now-robot_stamp_>.20 || now<robot_stamp_-.10)
        throw nav2_core::PlannerException("Predictive control state unavailable");
      map=static_map_;observations=observations_;
      rx=robot_x_;ry=robot_y_;ra=robot_yaw_;rv=robot_v_;rw=robot_w_;
    }
    std::vector<std::vector<Point>> futures;
    for(size_t i=0;i<observations.size();++i) {
      if(!observations[i].valid || now-observations[i].stamp>.25 || observations[i].stamp>now+.10)
        throw nav2_core::PlannerException("Predictive obstacle state stale");
      futures.push_back(forecast(motions_[i],observations[i],now,2.30));
    }
    nav_msgs::msg::Path plan;
    double lookahead,cruise;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan=global_plan_;lookahead=std::clamp(std::abs(rv)*lookahead_time_,min_lookahead_dist_,max_lookahead_dist_);
      cruise=desired_linear_vel_;
    }
    if(plan.poses.empty() || plan.header.frame_id!="map")
      throw nav2_core::PlannerException("Predictive control needs a map-frame path");
    auto footprint=costmap_ros_->getRobotFootprint();double hx=0,hy=0;
    for(auto p:footprint) {hx=std::max(hx,std::abs(p.x));hy=std::max(hy,std::abs(p.y));}
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map.get());
    const double px=pose.pose.position.x,py=pose.pose.position.y,pa=tf2::getYaw(pose.pose.orientation);
    const double transform_angle=wrap(ra-pa),tc=std::cos(transform_angle),ts=std::sin(transform_angle);
    // Static map uses the localization frame; actor geometry uses exact Gazebo
    // coordinates. Align each rollout using the current robot correspondence.
    auto safe=[&](double x,double y,double yaw,double t) {
      double wx=rx+(x-px)*tc-(y-py)*ts,wy=ry+(x-px)*ts+(y-py)*tc;
      unsigned mx,my;
      // The imported static map and simulator actors share world coordinates.
      // Check the actual footprint, including any localization heading error.
      if(!map->worldToMap(wx,wy,mx,my) || map->getCost(mx,my)>=253 ||
        checker.footprintCostAtPose(wx,wy,yaw+transform_angle,footprint)>=253) return false;
      for(size_t i=0;i<futures.size();++i) {
        // Include a 0.15 s state/actuation allowance around each future instant.
        for(double dt=-.15;dt<=.1501;dt+=.075) {
          size_t k=std::min(futures[i].size()-1,size_t(std::max(0.,t+dt)/.025));
          if(overlaps(wx,wy,yaw+transform_angle,hx,hy,futures[i][k],motions_[i].size/2+.10)) return false;
        }
      }
      return true;
    };
    const double nominal_v=command.twist.linear.x,nominal_w=command.twist.angular.z;
    const bool rotation=std::abs(nominal_v)<.001;
    auto rollout=[&](double cv,double cw,double scale) {
      double x=px,y=py,a=pa,v=rv,w=rw;
      size_t nearest=0;
      for(double t=.05;t<=2.0001;t+=.05) {
        double target_v=cv,target_w=cw;
        if(t>.30) {
          if(rotation) {
            size_t carrot=0;
            while(carrot+1<plan.poses.size() && std::hypot(plan.poses[carrot].pose.position.x-x,
              plan.poses[carrot].pose.position.y-y)<lookahead) ++carrot;
            double target=std::hypot(plan.poses.back().pose.position.x-x,plan.poses.back().pose.position.y-y)<.20 ?
              tf2::getYaw(plan.poses.back().pose.orientation) :
              std::atan2(plan.poses[carrot].pose.position.y-y,plan.poses[carrot].pose.position.x-x);
            const double error=wrap(target-a);
            target_v=0;target_w=std::copysign(std::min(4.,std::sqrt(20.*std::abs(error))),error);
          } else {
            double distance=1e9;
            for(size_t i=nearest;i<plan.poses.size();++i) {
              double d=std::hypot(plan.poses[i].pose.position.x-x,plan.poses[i].pose.position.y-y);
              if(d<distance) {distance=d;nearest=i;}
              else if(i>nearest+12) break;
            }
            size_t carrot=nearest;
            while(carrot+1<plan.poses.size() && std::hypot(plan.poses[carrot].pose.position.x-x,
                  plan.poses[carrot].pose.position.y-y)<lookahead) ++carrot;
            double dx=plan.poses[carrot].pose.position.x-x,dy=plan.poses[carrot].pose.position.y-y;
            double curvature=2.*(-dx*std::sin(a)+dy*std::cos(a))/std::max(.01,dx*dx+dy*dy);
            double radius=1./std::max(.001,std::abs(curvature));
            double remain=std::hypot(plan.poses.back().pose.position.x-x,plan.poses.back().pose.position.y-y);
            target_v=scale*std::min(cruise,std::max(.08,cruise*remain/1.2));
            if(radius<1.1) target_v=std::min(target_v,std::max(.18,cruise*radius/1.1));
            target_w=std::clamp(target_v*curvature,-4.,4.);
          }
        }
        v+=std::clamp(target_v-v,-5.*.05,3.5*.05);
        w+=std::clamp(target_w-w,-10.*.05,10.*.05);
        const double middle=a+w*.025;
        x+=v*std::cos(middle)*.05;y+=v*std::sin(middle)*.05;a=wrap(a+w*.05);
        if(!safe(x,y,a,t)) return false;
      }
      return true;
    };
    double chosen_v=nominal_v,chosen_w=nominal_w;bool found=rollout(nominal_v,nominal_w,1.);
    if(!found && !rotation) {
      double best=1e9;
      for(double scale:{1.,.85,.65,.45,.25}) for(double offset:{0.,-.25,.25,-.50,.50}) {
        double v=nominal_v*scale,w=std::clamp(nominal_w*scale+offset,-4.,4.);
        double score=(1.-scale)*2.+std::abs(offset)*.8;
        if(score<best && rollout(v,w,scale)) {best=score;chosen_v=v;chosen_w=w;found=true;}
      }
    }
    if(!found) {
      RCLCPP_WARN_THROTTLE(parent_->get_logger(),*parent_->get_clock(),500,"No collision-free short-horizon command; request replanning");
      throw nav2_core::PlannerException("No safe predictive command");
    }
    command.twist.linear.x=chosen_v;command.twist.angular.z=chosen_w;
    std_msgs::msg::String debug;std::ostringstream stream;
    stream<<"{\"sim_time\":"<<now<<",\"nominal\":["<<nominal_v<<","<<nominal_w<<"],\"selected\":["<<chosen_v<<","<<chosen_w<<"]}";
    debug.data=stream.str();debug_pub_->publish(debug);
    return command;
  }
};
}
PLUGINLIB_EXPORT_CLASS(aic_timed_planner::TimeAwareController,nav2_core::Controller)