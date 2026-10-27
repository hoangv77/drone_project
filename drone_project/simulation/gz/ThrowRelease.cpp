#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <atomic>
#include <chrono>

// A finite hand push supplies the configured release momentum through real
// forces. IMU and EKF therefore observe the acceleration before free flight.
class ThrowRelease : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & e, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(e);
    pose_ = sdf->Get<gz::math::Pose3d>("hold_pose");
    release_rpy_ = sdf->Get<gz::math::Vector3d>("release_rpy");
    velocity_ = sdf->Get<gz::math::Vector3d>("release_velocity");
    duration_ = sdf->Get<double>("push_duration");
    if (duration_ <= 0) {throw std::invalid_argument("push_duration must be positive");}
    released_pub_ = node_.Advertise<gz::msgs::Double>("/phase6/released");
    node_.Advertise("/phase6/release", &ThrowRelease::Release, this);
  }
  bool Release(const gz::msgs::Boolean & request, gz::msgs::Boolean & reply)
  {
    bool expected = false;
    reply.set_data(request.data() && requested_.compare_exchange_strong(expected, true));
    return true;
  }
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || released_ || !requested_) {return;}
    if (requested_ && !orienting_) {
      pose_.Rot() = gz::math::Quaterniond(release_rpy_);
      // Clear the hand pedestal when applying the release orientation.
      pose_.Pos().Z() += 0.05;
      orienting_ = true;
      model_.SetWorldPoseCmd(ecm, pose_);
      push_start_ = info.simTime + 2 * info.dt;
      return;
    }
    if (info.simTime < push_start_) {return;}
    if (std::chrono::duration<double>(info.simTime - push_start_).count() >= duration_) {
      released_ = true;
      gz::msgs::Double event;
      event.set_data(std::chrono::duration<double>(info.simTime).count());
      released_pub_.Publish(event);
      return;
    }
    ecm.Each<gz::sim::components::Link, gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity & e, const auto *, const auto * parent) {
        if (parent->Data() == model_.Entity()) {
          const auto * inertia = ecm.Component<gz::sim::components::Inertial>(e);
          if (inertia) {
            const double mass = inertia->Data().MassMatrix().Mass();
            gz::sim::Link(e).AddWorldForce(ecm,
              mass * (velocity_ / duration_ + gz::math::Vector3d(0, 0, 9.8)));
          }
        }
        return true;
      });
  }
private:
  gz::sim::Model model_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher released_pub_;
  gz::math::Pose3d pose_;
  gz::math::Vector3d velocity_, release_rpy_;
  std::atomic<bool> requested_{false};
  bool released_{false}, orienting_{false};
  double duration_{0.2};
  std::chrono::steady_clock::duration push_start_{};
};
GZ_ADD_PLUGIN(ThrowRelease, gz::sim::System, ThrowRelease::ISystemConfigure, ThrowRelease::ISystemPreUpdate)
