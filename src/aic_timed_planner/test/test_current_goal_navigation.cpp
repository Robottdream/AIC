#include <gtest/gtest.h>
#include "aic_timed_planner/current_goal_navigation.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <deque>
using BT::NodeStatus;
using Pose=geometry_msgs::msg::PoseStamped;
using Path=nav_msgs::msg::Path;

struct Script {
  std::deque<NodeStatus> plans;
  int planned=0,followed=0,plan_halts=0,follow_halts=0;
  bool write_path=true,wrong_goal=false;
  NodeStatus following=NodeStatus::RUNNING;
};
class Plan : public BT::StatefulActionNode {
  std::shared_ptr<Script> script_;
  NodeStatus step() {
    ++script_->planned;
    auto status=script_->plans.empty() ? NodeStatus::FAILURE : script_->plans.front();
    if(!script_->plans.empty()) script_->plans.pop_front();
    if(status==NodeStatus::SUCCESS && script_->write_path) {
      auto goal=config().blackboard->get<Pose>("goal");
      if(script_->wrong_goal) goal.pose.position.x+=1.;
      Path p;p.header=goal.header;p.poses.push_back(goal);setOutput("path",p);
    }
    return status;
  }
public:
  Plan(const std::string & n,const BT::NodeConfiguration & c):BT::StatefulActionNode(n,c),
    script_(c.blackboard->get<std::shared_ptr<Script>>("script")) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<Path>("path")};}
  NodeStatus onStart() override {return step();}
  NodeStatus onRunning() override {return step();}
  void onHalted() override {++script_->plan_halts;}
};
class Follow : public BT::StatefulActionNode {
  std::shared_ptr<Script> script_;
  NodeStatus step() {
    ++script_->followed;Path p;
    EXPECT_TRUE(getInput("path",p));EXPECT_FALSE(p.poses.empty());
    return script_->following;
  }
public:
  Follow(const std::string & n,const BT::NodeConfiguration & c):BT::StatefulActionNode(n,c),
    script_(c.blackboard->get<std::shared_ptr<Script>>("script")) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<Path>("path")};}
  NodeStatus onStart() override {return step();}
  NodeStatus onRunning() override {return step();}
  void onHalted() override {++script_->follow_halts;}
};
class CurrentGoalTest : public ::testing::Test {
protected:
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board=BT::Blackboard::create();
  std::shared_ptr<Script> script=std::make_shared<Script>();
  Pose goal;
  BT::Tree tree;
  void SetUp() override {
    goal.header.frame_id="map";goal.pose.position.x=2.;goal.pose.orientation.w=1.;
    board->set("goal",goal);board->set("script",script);
    factory.registerNodeType<aic_timed_planner::CurrentGoalNavigation>("CurrentGoalNavigation");
    factory.registerNodeType<Plan>("Plan");factory.registerNodeType<Follow>("Follow");
    tree=factory.createTreeFromText(R"(
      <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
        <CurrentGoalNavigation goal="{goal}" candidate_path="{candidate}" path="{path}" hz="1000000000">
          <Plan path="{candidate}"/><Follow path="{path}"/>
        </CurrentGoalNavigation>
      </BehaviorTree></root>)",board);
  }
};
TEST_F(CurrentGoalTest, FirstSearchFailureOrRunningNeverTicksFollower) {
  Path old;old.header=goal.header;old.poses.push_back(goal);
  board->set("path",old);board->set("candidate",old);
  script->plans={NodeStatus::RUNNING,NodeStatus::FAILURE};
  EXPECT_EQ(tree.tickRoot(),NodeStatus::RUNNING);EXPECT_EQ(script->followed,0);
  EXPECT_EQ(tree.tickRoot(),NodeStatus::FAILURE);EXPECT_EQ(script->followed,0);
  EXPECT_TRUE(board->get<Path>("path").poses.empty());
}
TEST_F(CurrentGoalTest, RollingFailureKeepsCurrentGoalPathAndControllerRunning) {
  script->plans={NodeStatus::SUCCESS,NodeStatus::FAILURE,NodeStatus::RUNNING};
  EXPECT_EQ(tree.tickRoot(),NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickRoot(),NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickRoot(),NodeStatus::RUNNING);
  EXPECT_EQ(script->followed,3);EXPECT_EQ(script->follow_halts,0);
  EXPECT_FALSE(board->get<Path>("path").poses.empty());
}
TEST_F(CurrentGoalTest, ChangedGoalHaltsControllerAndInflightSearchBeforeFirstNewPlan) {
  script->plans={NodeStatus::SUCCESS,NodeStatus::RUNNING,NodeStatus::FAILURE};
  tree.tickRoot();tree.tickRoot();
  goal.pose.position.x=5.;board->set("goal",goal);
  EXPECT_EQ(tree.tickRoot(),NodeStatus::FAILURE);
  EXPECT_EQ(script->followed,2);EXPECT_EQ(script->follow_halts,1);
  EXPECT_EQ(script->plan_halts,1);EXPECT_TRUE(board->get<Path>("path").poses.empty());
}
TEST_F(CurrentGoalTest, SameCoordinatesNewRequestStampStillRequiresAPlan) {
  script->plans={NodeStatus::SUCCESS,NodeStatus::FAILURE};tree.tickRoot();
  goal.header.stamp.sec=1;board->set("goal",goal);
  EXPECT_EQ(tree.tickRoot(),NodeStatus::FAILURE);EXPECT_EQ(script->followed,1);
}
TEST_F(CurrentGoalTest, HaltAndRetryCannotReusePreviousExecutionPath) {
  script->plans={NodeStatus::SUCCESS,NodeStatus::FAILURE};tree.tickRoot();tree.haltTree();
  EXPECT_EQ(tree.tickRoot(),NodeStatus::FAILURE);EXPECT_EQ(script->followed,1);
}
TEST_F(CurrentGoalTest, SuccessWithoutANewPathAndWrongEndpointFailClosed) {
  script->write_path=false;script->plans={NodeStatus::SUCCESS};
  EXPECT_EQ(tree.tickRoot(),NodeStatus::FAILURE);EXPECT_EQ(script->followed,0);
  script->write_path=true;script->wrong_goal=true;script->plans={NodeStatus::SUCCESS};
  EXPECT_EQ(tree.tickRoot(),NodeStatus::FAILURE);EXPECT_EQ(script->followed,0);
}
TEST_F(CurrentGoalTest, FollowCompletionClearsThePlanAndStopsPendingSearch) {
  script->plans={NodeStatus::SUCCESS,NodeStatus::RUNNING};tree.tickRoot();
  script->following=NodeStatus::SUCCESS;
  EXPECT_EQ(tree.tickRoot(),NodeStatus::SUCCESS);EXPECT_EQ(script->plan_halts,1);
  EXPECT_TRUE(board->get<Path>("path").poses.empty());
}
