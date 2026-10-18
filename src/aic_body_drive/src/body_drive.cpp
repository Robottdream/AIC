// Body-force velocity servo inspired by rmoss MecanumDrive2; independent
// Gazebo Classic implementation. No pose/velocity assignment or wheel motors.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>

namespace gazebo {
class BodyDrive : public ModelPlugin {
  struct Servo {
    double kp, ki, limit, integral = 0;
    double Step(double error, double dt) {
      const double candidate = std::clamp(integral + ki * error * dt, -limit, limit);
      const double proposed = kp * error + candidate;
      // Conditional integration: saturation must not wind up the integral.
      if (std::abs(proposed) <= limit || error * proposed < 0) integral = candidate;
      return std::clamp(kp * error + integral, -limit, limit);
    }
    void Reset() { integral = 0; }
  };
  physics::ModelPtr model_;
  physics::LinkPtr body_;
  physics::JointPtr left_, right_;
  gazebo_ros::Node::SharedPtr node_;
  event::ConnectionPtr update_;
  event::ConnectionPtr update_end_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::mutex mutex_;
  std::atomic<double> sim_time_{0};
  double v_ = 0, w_ = 0, received_ = -1e9, previous_ = 0, published_ = -1e9;
  double timeout_, max_v_, max_w_, period_;
  double lateral_acceleration_=3.,tilt_stop_=.08726646259971647,last_log_=-1e9;
  std::atomic<bool> safety_latched_{false};
  bool pending_snapshot_=false;
  bool pause_requested_=false,keep_contacts_owned_=false;
  std::deque<std::string> history_;
  std::string odom_frame_, base_frame_;
  Servo longitudinal_{300, 150, 150}, lateral_{1000, 300, 200}, yaw_{200, 100, 100};

  static double Positive(sdf::ElementPtr sdf, const std::string &key, double fallback) {
    const double value = sdf->Get<double>(key, fallback).first;
    if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Invalid body drive parameter: " + key);
    return value;
  }
  void Command(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      v_ = w_ = 0; received_ = -1e9;
      return;
    }
    // Differential chassis: accept forward velocity and yaw, no lateral target.
    v_ = std::clamp(msg->linear.x, -max_v_, max_v_);
    w_ = std::clamp(msg->angular.z, -max_w_, max_w_);
    received_ = sim_time_.load();
  }
  void Update(const common::UpdateInfo &info) {
    const double now = info.simTime.Double();
    sim_time_.store(now);
    const double dt = now - previous_;
    previous_ = now;
    if (dt <= 0 || dt > .1) {
      Reset();
      published_ = now - period_;
      return;
    }
    double target_v, target_w;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool fresh = now >= received_ && now - received_ <= timeout_;
      target_v = fresh ? v_ : 0;
      target_w = fresh ? w_ : 0;
    }
    const auto pose = body_->WorldPose();
    const double heading = pose.Rot().Yaw();
    const double c = std::cos(heading), s = std::sin(heading);
    const auto velocity = body_->WorldLinearVel();
    const auto angular = body_->WorldAngularVel();
    const double forward = c * velocity.X() + s * velocity.Y();
    const double sideways = -s * velocity.X() + c * velocity.Y();
    const double roll=pose.Rot().Roll(),pitch=pose.Rot().Pitch();
    if(!safety_latched_ && (!std::isfinite(roll) || !std::isfinite(pitch) ||
       !std::isfinite(forward) || !std::isfinite(sideways) ||
       !std::isfinite(angular.Z()) || std::max(std::abs(roll),std::abs(pitch))>tilt_stop_)) {
      safety_latched_=true;pending_snapshot_=true;
      auto manager=model_->GetWorld()->Physics()->GetContactManager();
      keep_contacts_owned_=!manager->NeverDropContacts();
      manager->SetNeverDropContacts(true);
      longitudinal_.Reset();lateral_.Reset();yaw_.Reset();
      RCLCPP_ERROR(node_->get_logger(),
        "STABILITY_FAULT sim=%.6f xyz=(%.5f,%.5f,%.5f) roll/pitch=(%.6f,%.6f): drive disabled; preserving paused scene",
        now,pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z(),roll,pitch);
    }
    // The smoother limits v and yaw independently. Enforce their combined
    // demand here too, including direct /cmd_vel and braking transients.
    const double demand=std::abs(target_v*target_w);
    if(demand>lateral_acceleration_) {
      const double scale=std::sqrt(lateral_acceleration_/demand);
      target_v*=scale;target_w*=scale;
    }
    const double steering=lateral_acceleration_/std::max({.01,std::abs(forward),std::abs(target_v)});
    target_w=std::clamp(target_w,-steering,steering);
    if(safety_latched_) target_v=target_w=0.;
    const double fx = safety_latched_ ? 0. : longitudinal_.Step(target_v - forward, dt);
    const double fy = safety_latched_ ? 0. : lateral_.Step(-sideways, dt);
    const double tz = safety_latched_ ? 0. : yaw_.Step(target_w - angular.Z(), dt);
    // World-horizontal forces at the body's CoG; retain gravity, roll/pitch,
    // collision/contact forces. Never inject vertical force or tilt correction.
    body_->AddForce({c * fx - s * fy, s * fx + c * fy, 0});
    body_->AddTorque({0, 0, tz});
    if (now - published_ < period_) return;
    published_ = now;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = info.simTime.sec; stamp.nanosec = info.simTime.nsec;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp; odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    const auto local_v = pose.Rot().RotateVectorReverse(velocity);
    const auto local_w = pose.Rot().RotateVectorReverse(angular);
    odom.twist.twist.linear.x = local_v.X(); odom.twist.twist.linear.y = local_v.Y(); odom.twist.twist.linear.z = local_v.Z();
    odom.twist.twist.angular.x = local_w.X(); odom.twist.twist.angular.y = local_w.Y(); odom.twist.twist.angular.z = local_w.Z();
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header; transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pose.Pos().X();
    transform.transform.translation.y = pose.Pos().Y();
    transform.transform.translation.z = pose.Pos().Z();
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_->sendTransform(transform);
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {left_->GetName(), right_->GetName()};
    joints.position = {left_->Position(0), right_->Position(0)};
    joints.velocity = {left_->GetVelocity(0), right_->GetVelocity(0)};
    joints_pub_->publish(joints);
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = stamp;
    wrench.header.frame_id = "world";
    wrench.wrench.force.x = c*fx-s*fy; wrench.wrench.force.y = s*fx+c*fy; wrench.wrench.torque.z = tz;
    wrench_pub_->publish(wrench);
    std::ostringstream state;
    state.precision(10);
    state<<"{\"sim\":"<<now<<",\"xyz\":["<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()
      <<"],\"rpy\":["<<roll<<","<<pitch<<","<<heading<<"],\"velocity\":["<<forward<<","<<sideways<<","<<angular.Z()
      <<"],\"target\":["<<target_v<<","<<target_w<<"],\"wrench\":["<<fx<<","<<fy<<","<<tz<<"]}";
    history_.push_back(state.str());if(history_.size()>150) history_.pop_front();
    if(now-last_log_>=1.) {
      RCLCPP_INFO(node_->get_logger(),"BODY_STATE %s",history_.back().c_str());last_log_=now;
    }
    if(pending_snapshot_) {
      pending_snapshot_=false;
      for(const auto & sample:history_) RCLCPP_ERROR(node_->get_logger(),"STABILITY_TRACE %s",sample.c_str());
      for(const auto & other:model_->GetWorld()->Models()) {
        if(other==model_) continue;
        const auto p=other->WorldPose().Pos();
        if((p-pose.Pos()).Length()<3.)
          RCLCPP_ERROR(node_->get_logger(),"STABILITY_NEIGHBOR %s xyz=(%.5f,%.5f,%.5f)",other->GetName().c_str(),p.X(),p.Y(),p.Z());
      }
      pause_requested_=true;
    }
  }
 public:
  ~BodyDrive() override { update_end_.reset();update_.reset(); command_sub_.reset(); }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    body_ = model->GetLink(sdf->Get<std::string>("body_link", "base_footprint").first);
    left_ = model->GetJoint("left_wheel_joint"); right_ = model->GetJoint("right_wheel_joint");
    if (!body_ || !left_ || !right_) gzthrow("BodyDrive requires body and both wheel joints");
    timeout_ = Positive(sdf, "command_timeout", .5);
    max_v_ = Positive(sdf, "max_linear_speed", 3.5);
    max_w_ = Positive(sdf, "max_angular_speed", 4.0);
    period_ = 1. / Positive(sdf, "publish_rate", 30.);
    lateral_acceleration_=Positive(sdf,"max_lateral_acceleration",3.);
    tilt_stop_=Positive(sdf,"tilt_stop_radians",.08726646259971647);
    for (const auto &entry : {std::make_pair("forward", &longitudinal_), std::make_pair("lateral", &lateral_), std::make_pair("yaw", &yaw_)}) {
      entry.second->kp = Positive(sdf, std::string(entry.first) + "_kp", entry.second->kp);
      entry.second->ki = Positive(sdf, std::string(entry.first) + "_ki", entry.second->ki);
      entry.second->limit = Positive(sdf, std::string(entry.first) + "_limit", entry.second->limit);
    }
    odom_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
    // No wheel motor can coexist with the body wrench servo.
    left_->SetParam("fmax", 0, 0.0); right_->SetParam("fmax", 0, 0.0);
    node_ = gazebo_ros::Node::Get(sdf);
    command_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1), std::bind(&BodyDrive::Command, this, std::placeholders::_1));
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    joints_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("body_drive/wrench", rclcpp::QoS(10));
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    previous_ = model->GetWorld()->SimTime().Double(); sim_time_.store(previous_);
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BodyDrive::Update, this, std::placeholders::_1));
    update_end_=event::Events::ConnectWorldUpdateEnd([this]() {
      if(!pause_requested_) return;
      pause_requested_=false;
      const auto manager=model_->GetWorld()->Physics()->GetContactManager();
      const auto & contacts=manager->GetContacts();
      unsigned relevant=0;
      for(unsigned i=0;i<manager->GetContactCount() && i<contacts.size();++i) {
        const auto contact=contacts[i];
        if(!contact || !contact->collision1 || !contact->collision2) continue;
        const auto first=contact->collision1->GetScopedName(),second=contact->collision2->GetScopedName();
        if(first.find(model_->GetScopedName()+"::")!=0 && second.find(model_->GetScopedName()+"::")!=0) continue;
        double depth=0.;for(int j=0;j<contact->count;++j) depth=std::max(depth,contact->depths[j]);
        RCLCPP_ERROR(node_->get_logger(),"STABILITY_CONTACT %s / %s depth=%.8f",first.c_str(),second.c_str(),depth);++relevant;
      }
      RCLCPP_ERROR(node_->get_logger(),"STABILITY_PAUSED contacts=%u; snapshot in Gazebo log; restart required",relevant);
      model_->GetWorld()->SetPaused(true);
    });
    RCLCPP_INFO(node_->get_logger(), "Body force drive active: force limits %.1f/%.1f N, yaw %.1f Nm; passive wheels", longitudinal_.limit, lateral_.limit, yaw_.limit);
    RCLCPP_INFO(node_->get_logger(),"Coupled turn limit %.2f m/s^2; tilt pause %.2f deg; BODY_STATE history enabled",lateral_acceleration_,tilt_stop_*180./3.14159265358979323846);
  }
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    v_ = w_ = 0; received_ = -1e9;
    longitudinal_.Reset(); lateral_.Reset(); yaw_.Reset();
    safety_latched_=false;pending_snapshot_=false;history_.clear();last_log_=-1e9;
    pause_requested_=false;
    if(keep_contacts_owned_ && model_) model_->GetWorld()->Physics()->GetContactManager()->SetNeverDropContacts(false);
    keep_contacts_owned_=false;
  }
};
GZ_REGISTER_MODEL_PLUGIN(BodyDrive)
}  // namespace gazebo
