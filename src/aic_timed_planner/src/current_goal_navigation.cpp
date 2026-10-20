#include "aic_timed_planner/current_goal_navigation.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<aic_timed_planner::CurrentGoalNavigation>("CurrentGoalNavigation");
}
