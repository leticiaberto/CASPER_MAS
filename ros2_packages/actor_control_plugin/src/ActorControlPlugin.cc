#include <mutex>
#include <string>

// correct includes for Gazebo Fortress (ignition-gazebo6)
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Actor.hh>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/plugin/Register.hh>

#include <sdf/Element.hh>

using namespace ignition;
using namespace ignition::gazebo;

class ActorControlPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
public:

  void Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &) override
  {
    this->actorEntity = _entity;

    std::string topic = "/actor/cmd_pose";
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");

    this->node.Subscribe(topic,
      &ActorControlPlugin::OnPoseMsg, this);

    this->moving = false;
    this->targetPose = math::Pose3d::Zero;

    ignmsg << "ActorControlPlugin loaded, listening on: "
           << topic << std::endl;
  }

  void PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm) override
  {
    if (_info.paused) return;
    if (!this->moving) return;

    auto poseComp =
      _ecm.Component<components::Pose>(this->actorEntity);
    if (!poseComp) return;

    math::Pose3d current = poseComp->Data();

    std::lock_guard<std::mutex> lock(this->mutex);
    math::Pose3d next =
      this->InterpolateToward(current, this->targetPose, 0.05);

    _ecm.SetComponentData<components::Pose>(
      this->actorEntity, next);

    if (current.Pos().Distance(this->targetPose.Pos()) < 0.05)
      this->moving = false;
  }

private:

  void OnPoseMsg(const ignition::msgs::Pose &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->targetPose = ignition::msgs::Convert(_msg);
    this->moving = true;
  }

  math::Pose3d InterpolateToward(
    const math::Pose3d &_from,
    const math::Pose3d &_to,
    double _step)
  {
    math::Vector3d dir = _to.Pos() - _from.Pos();
    if (dir.Length() < _step)
      return _to;

    dir.Normalize();
    return math::Pose3d(_from.Pos() + dir * _step, _to.Rot());
  }

  Entity actorEntity;
  ignition::transport::Node node;
  math::Pose3d targetPose;
  bool moving;
  std::mutex mutex;
};

// correct macro for Fortress
IGNITION_ADD_PLUGIN(
  ActorControlPlugin,
  ignition::gazebo::System,
  ActorControlPlugin::ISystemConfigure,
  ActorControlPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ActorControlPlugin, "ActorControlPlugin")