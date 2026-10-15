#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/ode/ode.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace gazebo {
class ContactLimitProbe : public WorldPlugin {
  physics::WorldPtr world_;
  event::ConnectionPtr update_, before_;
  std::ofstream log_;
  double previous_ = -1;
  bool parameter_ = false, finite_ = false;

  static dBodyID Body(const physics::LinkPtr &link) {
    auto ode = boost::dynamic_pointer_cast<physics::ODELink>(link);
    return ode ? ode->GetODEId() : nullptr;
  }

  // ODE's finite-rotation axis is a WORLD vector and must follow the hinge.
  void Before(const common::UpdateInfo &) {
    if (!finite_) return;
    auto model = world_->ModelByName("six_arm");
    if (!model) return;
    for (const auto &side : {"left", "right"}) {
      const std::string prefix(side);
      auto link = model->GetLink(prefix + "_wheel_link");
      auto joint = model->GetJoint(prefix + "_wheel_joint");
      auto id = Body(link);
      if (!id || !joint) continue;
      const auto axis = joint->GlobalAxis(0);
      dBodySetFiniteRotationMode(id, 1);
      dBodySetFiniteRotationAxis(id, axis.X(), axis.Y(), axis.Z());
    }
  }

  void Number(double value) {
    if (std::isfinite(value)) log_ << value;
    else log_ << "null";
  }

  static double AxisError(const physics::LinkPtr &wheel,
                          const ignition::math::Vector3d &expected) {
    // URDF wheel joints: origin RPY=0; both spin about link-local +Y.
    auto actual = wheel->WorldPose().Rot().RotateVector({0, 1, 0});
    const double cosine = std::abs(actual.Dot(expected) /
                                  (actual.Length() * expected.Length()));
    return std::acos(std::max(0.0, std::min(1.0, cosine))) * 180 / std::acos(-1.0);
  }

 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    finite_ = sdf->Get<bool>("finite_rotation", false).first;
    log_.open(sdf->Get<std::string>("output_file"));
    if (!log_) gzthrow("Cannot open native probe output");
    log_ << std::setprecision(12);
    before_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ContactLimitProbe::Before, this, std::placeholders::_1));
    update_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&ContactLimitProbe::Update, this));
  }

  void Update() {
    const double t = world_->SimTime().Double();
    if (t - previous_ < .0099) return;
    previous_ = t;
    auto model = world_->ModelByName("six_arm");
    if (!model) return;
    auto base = model->GetLink("base_link");
    if (!base) base = model->GetLink("base_footprint");
    auto left = model->GetLink("left_wheel_link");
    auto right = model->GetLink("right_wheel_link");
    auto lj = model->GetJoint("left_wheel_joint");
    auto rj = model->GetJoint("right_wheel_joint");
    if (!base || !left || !right || !lj || !rj) return;
    auto id = Body(base), lid = Body(left), rid = Body(right);
    auto physics = boost::dynamic_pointer_cast<physics::ODEPhysics>(world_->Physics());
    if (!id || !lid || !rid || !physics) return;
    auto wid = dBodyGetWorld(id);
    if (!parameter_) {
      log_ << "{\"kind\":\"parameters\",\"sim\":" << t
           << ",\"actual_contact_max_correcting_vel\":" << dWorldGetContactMaxCorrectingVel(wid)
           << ",\"solver\":\"" << physics->GetStepType()
           << "\",\"max_step_size\":" << physics->GetMaxStepSize()
           << ",\"real_time_update_rate\":" << physics->GetRealTimeUpdateRate()
           << ",\"cfm\":" << dWorldGetCFM(wid)
           << ",\"quick_iterations\":" << dWorldGetQuickStepNumIterations(wid)
           << ",\"finite_requested\":" << (finite_ ? "true" : "false")
           << ",\"left_finite_mode\":" << dBodyGetFiniteRotationMode(lid)
           << ",\"right_finite_mode\":" << dBodyGetFiniteRotationMode(rid)
           << ",\"world_max_angular_speed\":";
      Number(dWorldGetMaxAngularSpeed(wid));
      log_ << ",\"base_max_angular_speed\":"; Number(dBodyGetMaxAngularSpeed(id));
      log_ << ",\"left_max_angular_speed\":"; Number(dBodyGetMaxAngularSpeed(lid));
      log_ << ",\"right_max_angular_speed\":"; Number(dBodyGetMaxAngularSpeed(rid));
      log_ << ",\"left_velocity_limit\":"; Number(lj->GetVelocityLimit(0));
      log_ << ",\"right_velocity_limit\":"; Number(rj->GetVelocityLimit(0));
      log_ << ",\"left_effort_limit\":"; Number(lj->GetEffortLimit(0));
      log_ << "}\n";
      log_.flush();
      parameter_ = true;
    }
    const auto pose = base->WorldPose();
    const double yaw = pose.Rot().Yaw();
    const auto velocity = base->WorldLinearVel(), angular = base->WorldAngularVel();
    const auto axis = pose.Rot().RotateVector({0, 1, 0});
    log_ << "{\"kind\":\"motion\",\"sim\":" << t
         << ",\"x\":" << pose.Pos().X() << ",\"y\":" << pose.Pos().Y()
         << ",\"z\":" << pose.Pos().Z() << ",\"yaw\":" << yaw
         << ",\"vx\":" << velocity.X() << ",\"vy\":" << velocity.Y()
         << ",\"tilt_deg\":" << std::max(std::abs(pose.Rot().Roll()), std::abs(pose.Rot().Pitch())) * 180 / std::acos(-1.0)
         << ",\"v\":" << velocity.X() * std::cos(yaw) + velocity.Y() * std::sin(yaw)
         << ",\"w\":" << angular.Z()
         << ",\"left_joint_w\":" << lj->GetVelocity(0)
         << ",\"right_joint_w\":" << rj->GetVelocity(0)
         << ",\"left_joint_pos\":" << lj->Position(0)
         << ",\"right_joint_pos\":" << rj->Position(0)
         << ",\"left_axis_error_deg\":" << AxisError(left, axis)
         << ",\"right_axis_error_deg\":" << AxisError(right, axis)
         << ",\"left_finite_mode\":" << dBodyGetFiniteRotationMode(lid)
         << ",\"right_finite_mode\":" << dBodyGetFiniteRotationMode(rid) << "}\n";
    if (int(t * 100) % 100 == 0) log_.flush();
  }
};
GZ_REGISTER_WORLD_PLUGIN(ContactLimitProbe)
}
