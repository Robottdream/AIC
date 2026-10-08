/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo_ros/node.hpp>
#include <memory>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <mutex>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "ros2_linkattacher/gazebo_link_attacher.hpp"   // INCLUDE HADER FILE.
#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.

// GLOBAL VARIABLE:
std::vector<JointSTRUCT> GV_joints;
JointSTRUCT GV_jointSTR;

// IsAttached variable:
// Gazebo breaks if -> An attachment request is done between 2 links, and the joint attachment has already been created and not removed!
// Therefore, we have added this variable to make sure the attachment is only requested when the previous attachment has already been removed.
bool IsAttached = false;

// JointName:
std::string JointName = "None";

namespace gazebo_ros
{

class GazeboLinkAttacherPrivate
{
public:

  // ATTACH (ROS2 service):
  void Attach(
    linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res);

  // DETACH (ROS2 service):
  void Detach(
    linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res);

  void AttachOnUpdate(
    linkattacher_msgs::srv::AttachLink::Request::SharedPtr request,
    linkattacher_msgs::srv::AttachLink::Response::SharedPtr response);
  void DetachOnUpdate(
    linkattacher_msgs::srv::DetachLink::Request::SharedPtr request,
    linkattacher_msgs::srv::DetachLink::Response::SharedPtr response);
  bool DispatchToUpdate(std::function<void()> action);
  void OnWorldUpdate();

  struct PendingOperation
  {
    std::function<void()> action;
    std::promise<void> done;
    std::atomic<bool> cancelled{false};
  };
  std::mutex pending_mutex_;
  std::deque<std::shared_ptr<PendingOperation>> pending_;
  gazebo::event::ConnectionPtr update_connection_;

  // World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  // ROS services to handle requests for attach/detach.
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_service_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_service_;

  // getJoint function:
  bool getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint);

};

GazeboLinkAttacher::GazeboLinkAttacher()
: impl_(std::make_unique<GazeboLinkAttacherPrivate>())
{
}

GazeboLinkAttacher::~GazeboLinkAttacher()
{
  impl_->update_connection_.reset();
}

void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  
  // Gazebo WORLD:
  impl_->world_ = _world;

  // Apply joint and model changes in Gazebo's update thread. Calling them
  // directly from ROS service threads can race ODE and crash gzserver.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    [this](const gazebo::common::UpdateInfo &) {impl_->OnWorldUpdate();});

  // ROS2 NODE:
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // ROS2 SERVICE SERVERS:
  impl_->attach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
    "ATTACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Attach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));
  impl_->detach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
    "DETACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Detach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

}

bool GazeboLinkAttacherPrivate::DispatchToUpdate(std::function<void()> action)
{
  auto operation = std::make_shared<PendingOperation>();
  operation->action = std::move(action);
  auto future = operation->done.get_future();
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.push_back(operation);
  }
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    operation->cancelled.store(true);
    return false;
  }
  return true;
}

void GazeboLinkAttacherPrivate::OnWorldUpdate()
{
  std::shared_ptr<PendingOperation> operation;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_.empty()) return;
    operation = pending_.front();
    pending_.pop_front();
  }
  if (!operation->cancelled.load()) operation->action();
  operation->done.set_value();
}

void GazeboLinkAttacherPrivate::Attach(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr request,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr response)
{
  if (!DispatchToUpdate([this, request, response]() {
      AttachOnUpdate(request, response);
    })) {
    response->success = false;
    response->message = "Gazebo update timed out while attaching";
  }
}

void GazeboLinkAttacherPrivate::Detach(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr request,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr response)
{
  if (!DispatchToUpdate([this, request, response]() {
      DetachOnUpdate(request, response);
    })) {
    response->success = false;
    response->message = "Gazebo update timed out while detaching";
  }
}

void GazeboLinkAttacherPrivate::AttachOnUpdate(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res)
{

  // CHECK if -> Joint already exists in GV_joints:
  /* THIS IS NO LONGER NEEDED, SINCE THE JOINT IS REMOVED AFTER DETACHING!
  JointSTRUCT j;
  if (this->getJoint(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j)){
    j.joint->Attach(j.l1, j.l2);
    _res->success = true;
    _res->message = "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
    return;
  }
  */

  // Get the first link:
  gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);
  if (!model1) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model1_name;
    return;
  }
  gazebo::physics::LinkPtr link1 = model1->GetLink(_req->link1_name);
  if (!link1) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link1_name;
    return;
  }

  // Get the second link:
  gazebo::physics::ModelPtr model2 = world_->ModelByName(_req->model2_name);
  if (!model2) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model2_name;
    return;
  }
  gazebo::physics::LinkPtr link2 = model2->GetLink(_req->link2_name);
  if (!link2) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link2_name;
    return;
  }

  if (IsAttached == true){

    _res->success = false;
    _res->message = "Both links have already been attached, aborting new attachment.";

  } else {

    // Create a fixed joint between the two links:
    JointName = _req->model1_name + "_" + _req->link1_name + "_" + _req->model2_name + "_" + _req->link2_name + "_joint";
    // The carried block must be rigidly attached to the gripper. A revolute
    // joint with zero limits is not a fixed joint, and axis 1 does not exist
    // on a one-DOF joint (Gazebo reported SetDamping out of bounds).
    gazebo::physics::JointPtr joint = model1->CreateJoint(JointName, "fixed", link1, link2);
    joint->Attach(link1, link2);
    joint->Load(link1, link2, ignition::math::Pose3d());
    joint->SetProvideFeedback(true);
    joint->Init();
    model1->Update();

    GV_jointSTR.model1 = _req->model1_name;
    GV_jointSTR.model2 = _req->model2_name;
    GV_jointSTR.link1 = _req->link1_name;
    GV_jointSTR.link2 = _req->link2_name;
    GV_jointSTR.m1 = model1;
    GV_jointSTR.m2 = model2;
    GV_jointSTR.l1 = link1;
    GV_jointSTR.l2 = link2;
    GV_jointSTR.joint = joint;
    
    GV_joints.push_back(GV_jointSTR);

    // Set the success and message in the response:
    _res->success = true;
    _res->message = "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";

    IsAttached = true;

  }

}

void GazeboLinkAttacherPrivate::DetachOnUpdate(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res)
{

  // CHECK if -> Joint already exists in GV_joints:
  JointSTRUCT j;
  if (this->getJoint(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j)){
    j.joint->Detach();
    _res->success = true;
    _res->message = "DETACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
    
    // (+) Remove joint --> This fixes the following problem: If the object to be attached is removed and spawned again, 
    // gazebo breaks when attaching it again, since the joint already existed. Joint must be REMOVED when detaching.
    const std::string joint_name = j.joint->GetName();
    j.m1->RemoveJoint(joint_name);
    GV_joints.erase(std::remove_if(GV_joints.begin(), GV_joints.end(),
      [&j](const JointSTRUCT &entry) {
        return entry.joint == j.joint;
      }), GV_joints.end());

    // A fixed-joint simulation can occasionally put a tiny cube far from the
    // gripper. Correct an implausible release pose before freezing the cube.
    const auto gripper_pose = j.l1->WorldPose();
    const auto cube_pose = j.m2->WorldPose();
    if ((cube_pose.Pos() - gripper_pose.Pos()).Length() > 0.75 ||
        std::abs(cube_pose.Pos().Z() - 0.75) > 0.3) {
      j.m2->SetWorldPose(ignition::math::Pose3d(
        ignition::math::Vector3d(gripper_pose.Pos().X(), gripper_pose.Pos().Y(), 0.75),
        ignition::math::Quaterniond::Identity));
    }
    j.l2->SetLinearVel(ignition::math::Vector3d::Zero);
    j.l2->SetAngularVel(ignition::math::Vector3d::Zero);
    j.l2->SetKinematic(true);

    IsAttached = false;
    
    return;
  } else {
    _res->success = false;
    _res->message = "DETACHED -- ERROR (Joint does not exist!): {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
  }

}

bool GazeboLinkAttacherPrivate::getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint)
  {
    JointSTRUCT j;
    for(std::vector<JointSTRUCT>::iterator it = GV_joints.begin(); it != GV_joints.end(); ++it){
      j = *it;
      if ((j.model1.compare(M1) == 0) && (j.model2.compare(M2) == 0) && (j.link1.compare(L1) == 0) && (j.link2.compare(L2) == 0)){
        joint = j;
        return true;
      }
    }
    return false;
  }

GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)

}  // namespace gazebo_ros