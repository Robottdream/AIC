#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <chrono>
#include <thread>

namespace {
bool planning_ok, following_done;
int follows;
class Plan : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("goal"),
    BT::OutputPort<std::string>("path"),BT::InputPort<std::string>("planner_id")};}
  BT::NodeStatus tick() override {
    if(!planning_ok) return BT::NodeStatus::FAILURE;
    setOutput("path",getInput<std::string>("goal").value());
    return BT::NodeStatus::SUCCESS;
  }
};
class Follow : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("path"),
    BT::InputPort<std::string>("controller_id")};}
  BT::NodeStatus onStart() override {return onRunning();}
  BT::NodeStatus onRunning() override {
    ++follows;
    EXPECT_EQ(getInput<std::string>("path").value(),config().blackboard->get<std::string>("goal"));
    return following_done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
};
void registerNodes(BT::BehaviorTreeFactory & factory) {
  factory.registerFromPlugin("libnav2_pipeline_sequence_bt_node.so");
  factory.registerFromPlugin("libnav2_recovery_node_bt_node.so");
  factory.registerFromPlugin("libnav2_rate_controller_bt_node.so");
  factory.registerNodeType<Plan>("ComputePathToPose");
  factory.registerNodeType<Follow>("FollowPath");
  factory.registerSimpleAction("ClearEntireCostmap",[](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
    {BT::InputPort<std::string>("service_name")});
  factory.registerSimpleCondition("GoalUpdated",[](BT::TreeNode &) {return BT::NodeStatus::FAILURE;});
}
}

TEST(NavigationTree, FailedNewGoalCannotExecutePersistedOldPath) {
  BT::BehaviorTreeFactory factory;registerNodes(factory);
  auto blackboard=BT::Blackboard::create();blackboard->set("goal",std::string("cargo"));
  auto tree=factory.createTreeFromFile(AIC_NAVIGATION_TREE,blackboard);
  planning_ok=true;following_done=true;follows=0;
  ASSERT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);
  ASSERT_GT(follows,0);tree.haltTree();
  blackboard->set("goal",std::string("area"));planning_ok=false;follows=0;
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::FAILURE);
  EXPECT_EQ(follows,0);
}

TEST(NavigationTree, FailedReplanKeepsValidPathForSameGoal) {
  BT::BehaviorTreeFactory factory;registerNodes(factory);
  auto blackboard=BT::Blackboard::create();blackboard->set("goal",std::string("area"));
  auto tree=factory.createTreeFromFile(AIC_NAVIGATION_TREE,blackboard);
  planning_ok=true;following_done=false;follows=0;
  ASSERT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);
  planning_ok=false;
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);
  EXPECT_GE(follows,2);tree.haltTree();
}
