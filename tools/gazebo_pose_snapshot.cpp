#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Quaternion.hh>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>

std::mutex mutex;
std::map<std::string, gazebo::msgs::Pose> poses;
std::set<std::string> names;
void callback(ConstPosesStampedPtr &message) {
  std::lock_guard<std::mutex> lock(mutex);
  for (int i = 0; i < message->pose_size(); ++i) {
    const auto &pose = message->pose(i);
    if (names.count(pose.name())) poses[pose.name()] = pose;
  }
}
int main(int argc, char **argv) {
  names.insert("six_arm");
  for (const auto &color : {"red", "blue"})
    for (int i = 1; i <= 5; ++i) names.insert(std::string(color) + "_cube_" + std::to_string(i));
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");
  auto sub = node->Subscribe("~/pose/info", callback);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  bool complete = false;
  while (std::chrono::steady_clock::now() < end) {
    { std::lock_guard<std::mutex> lock(mutex); complete = poses.size() == names.size(); }
    if (complete) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (complete) {
    std::lock_guard<std::mutex> lock(mutex);
    std::cout.precision(12);
    std::cout << "{";
    bool first = true;
    for (const auto &entry : poses) {
      if (!first) std::cout << ",";
      first = false;
      const auto &p = entry.second.position();
      const auto &o = entry.second.orientation();
      ignition::math::Quaterniond q(o.w(), o.x(), o.y(), o.z());
      const auto e = q.Euler();
      std::cout << "\"" << entry.first << "\":[" << p.x() << "," << p.y() << "," << p.z()
                << "," << e.X() << "," << e.Y() << "," << e.Z() << "]";
    }
    std::cout << "}\n";
  } else std::cerr << "Only received " << poses.size() << " of " << names.size() << " model poses\n";
  sub.reset();
  node->Fini();
  gazebo::client::shutdown();
  return complete ? 0 : 1;
}
