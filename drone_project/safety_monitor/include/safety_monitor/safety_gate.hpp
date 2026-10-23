#pragma once

#include <safety_monitor/msg/safety_status.hpp>

#include <chrono>
#include <mutex>
#include <stdexcept>

namespace safety_monitor
{
// Intended for the state machine's transition checks. Receipt uses the same
// steady clock as timeout evaluation, independent of /clock and ROS timestamps.
class SafetyGate
{
public:
  enum class Operation {Launch, AutonomousFlight, VisualLanding};
  using Clock = std::chrono::steady_clock;
  explicit SafetyGate(std::chrono::milliseconds max_age = std::chrono::milliseconds(500))
  : max_age_(max_age)
  {
    if (max_age_.count() <= 0) {throw std::invalid_argument("Safety max_age must be positive");}
  }

  void receive(const msg::SafetyStatus & status, Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = status;
    last_receive_ = now;
    received_ = true;
  }

  bool permits(Operation operation, bool allow_warning = false,
    Clock::time_point now = Clock::now()) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || now < last_receive_ || now - last_receive_ > max_age_ ||
      !status_.telemetry_healthy ||
      !(status_.level == msg::SafetyStatus::READY ||
      (allow_warning && status_.level == msg::SafetyStatus::WARNING)))
    {
      return false;
    }
    switch (operation) {
      case Operation::Launch: return status_.launch_permitted;
      case Operation::AutonomousFlight: return status_.autonomous_flight_permitted;
      case Operation::VisualLanding: return status_.visual_landing_permitted;
    }
    return false;
  }

private:
  std::chrono::milliseconds max_age_;
  mutable std::mutex mutex_;
  msg::SafetyStatus status_;
  Clock::time_point last_receive_{};
  bool received_{false};
};
}  // namespace safety_monitor
