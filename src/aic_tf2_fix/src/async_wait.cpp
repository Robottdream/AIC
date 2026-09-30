// BSD-3-Clause. Based on the asynchronous Buffer API in geometry2 0.25.23.
// See NOTICE for upstream attribution. No class layout or installed files change.
#include <tf2_ros/buffer.hpp>
#include <memory>
#include <mutex>

namespace tf2_ros
{
TransformStampedFuture Buffer::waitForTransform(
  const std::string & target, const std::string & source, const tf2::TimePoint & stamp,
  const tf2::Duration & timeout, TransformReadyCallback callback)
{
  if (!timer_interface_) {
    throw CreateTimerInterfaceException("timer interface not set before asynchronous TF wait");
  }
  auto promise = std::make_shared<std::promise<geometry_msgs::msg::TransformStamped>>();
  TransformStampedFuture future(promise->get_future());
  // A transform may arrive before addTransformableRequest returns. Defer that
  // completion until its timer is registered, without blocking the TF thread.
  struct Registration {
    std::mutex mutex;
    bool ready = false;
    bool arrived = false;
    tf2::TransformableResult result = tf2::TransformAvailable;
  };
  auto registration = std::make_shared<Registration>();
  auto finish = [this, promise, future, callback, target, source, stamp](
    tf2::TransformableRequestHandle request, tf2::TransformableResult result) {
      bool claimed = false;
      {
        std::lock_guard<std::mutex> lock(timer_to_request_map_mutex_);
        for (auto it = timer_to_request_map_.begin(); it != timer_to_request_map_.end(); ++it) {
          if (it->second == request) {
            timer_interface_->remove(it->first);
            timer_to_request_map_.erase(it);
            claimed = true;
            break;
          }
        }
      }
      if (!claimed) {return;}  // Timeout/cancel already consumed the registration.
      try {
        if (result != tf2::TransformAvailable) {
          throw tf2::LookupException("Asynchronous TF unavailable: " + source + " -> " + target);
        }
        promise->set_value(lookupTransform(target, source, stamp));
      } catch (...) {
        promise->set_exception(std::current_exception());
      }
      callback(future);
    };
  auto on_transform = [registration, finish](
    tf2::TransformableRequestHandle request, const std::string &, const std::string &,
    tf2::TimePoint, tf2::TransformableResult result) {
      {
        std::lock_guard<std::mutex> lock(registration->mutex);
        if (!registration->ready) {
          registration->arrived = true;
          registration->result = result;
          return;
        }
      }
      finish(request, result);
    };
  // Crucially, never hold timer_to_request_map_mutex_ while acquiring the
  // BufferCore request mutex. The TF callback acquires these in reverse order.
  const auto request = addTransformableRequest(on_transform, target, source, stamp);
  future.setHandle(request);
  if (request == 0 || request == UINT64_MAX) {
    try {
      if (request == UINT64_MAX) {throw tf2::LookupException("TF request is outside cache");}
      promise->set_value(lookupTransform(target, source, stamp));
    } catch (...) {
      promise->set_exception(std::current_exception());
    }
    callback(future);
    return future;
  }
  {
    std::lock_guard<std::mutex> lock(timer_to_request_map_mutex_);
    auto timer = timer_interface_->createTimer(clock_, timeout,
      std::bind(&Buffer::timerCallback, this, std::placeholders::_1, promise, future, callback));
    timer_to_request_map_[timer] = request;
  }
  bool arrived;
  tf2::TransformableResult result;
  {
    std::lock_guard<std::mutex> lock(registration->mutex);
    registration->ready = true;
    arrived = registration->arrived;
    result = registration->result;
  }
  if (arrived) {finish(request, result);}
  return future;
}
}  // namespace tf2_ros
