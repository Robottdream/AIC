// Concurrent async TF waits versus transform arrival; run with a wall timeout.
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

class Timers : public tf2_ros::CreateTimerInterface
{
public:
  tf2_ros::TimerHandle createTimer(rclcpp::Clock::SharedPtr, const tf2::Duration &,
    tf2_ros::TimerCallbackType cb) override {
    std::lock_guard<std::mutex> lock(mutex);
    auto id = ++counter;
    callbacks[id] = cb;
    std::this_thread::sleep_for(std::chrono::microseconds(20));
    return id;
  }
  void cancel(const tf2_ros::TimerHandle & id) override {remove(id);}
  void reset(const tf2_ros::TimerHandle &) override {}
  void remove(const tf2_ros::TimerHandle & id) override {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.erase(id);
  }
  void expire_all() {
    std::map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> copy;
    {std::lock_guard<std::mutex> lock(mutex); copy = callbacks;}
    for (auto & [id, cb] : copy) {cb(id);}
  }
private:
  std::mutex mutex;
  uint64_t counter = 0;
  std::map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> callbacks;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  auto timers = std::make_shared<Timers>();
  buffer.setCreateTimerInterface(timers);
  std::atomic<uint64_t> generation{1}, completed{0}, failed{0};
  std::atomic<bool> running{true};
  auto transform = [](uint64_t sequence) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom";
    t.child_frame_id = "base";
    t.header.stamp = rclcpp::Time(1000000000LL + static_cast<int64_t>(sequence)*1000000LL);
    t.transform.rotation.w = 1.0;
    return t;
  };
  buffer.setTransform(transform(1), "stress");
  std::thread writer([&]() {
    while (running) {
      auto sequence = ++generation;
      buffer.setTransform(transform(sequence), "stress");
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  });
  constexpr unsigned count = 20000;
  for (unsigned i = 0; i < count; ++i) {
    auto stamp = tf2::TimePoint(std::chrono::nanoseconds(
        1000000000LL + static_cast<int64_t>(generation.load()+2)*1000000LL));
    buffer.waitForTransform("odom", "base", stamp, std::chrono::seconds(1),
      [&](const tf2_ros::TransformStampedFuture & result) {
        try {result.get(); ++completed;} catch (...) {++failed;}
      });
  }
  for (unsigned i = 0; i < 1000 && completed+failed < count; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  timers->expire_all();
  running = false;
  writer.join();
  auto latest = buffer.lookupTransform("odom", "base", tf2::TimePointZero);
  // Also exercise timeout and immediate success after contention has stopped.
  buffer.waitForTransform("odom", "missing", tf2::TimePointZero, std::chrono::milliseconds(1),
    [&](const tf2_ros::TransformStampedFuture & result) {
      try {result.get();} catch (...) {++failed;}
    });
  timers->expire_all();
  buffer.waitForTransform("odom", "base", tf2::TimePointZero, std::chrono::seconds(1),
    [&](const tf2_ros::TransformStampedFuture & result) {result.get(); ++completed;});
  std::cout << "completed=" << completed << " failed=" << failed
            << " generations=" << generation << std::endl;
  rclcpp::shutdown();
  return completed+failed == count+2 && completed > count/2 ? 0 : 1;
}
