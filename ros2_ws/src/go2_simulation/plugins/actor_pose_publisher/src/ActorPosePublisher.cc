// World-level gz-sim system plugin that publishes the WorldPose of every
// actor in the world as a gz.msgs.Pose_V message. Actors do not appear in
// SceneBroadcaster's dynamic_pose/info because they are kinematic and lack
// the components SceneBroadcaster scans, so this plugin fills that gap for
// evaluation/ground-truth use.

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/msgs/pose_v.pb.h>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Actor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

namespace go2_simulation
{

class ActorPosePublisher
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    (void)_entity;
    (void)_ecm;

    std::string topic = "/world/default/actor_pose/info";
    double rateHz = 50.0;
    std::string frame = "world";

    if (_sdf->HasElement("topic")) {
      topic = _sdf->Get<std::string>("topic");
    }
    if (_sdf->HasElement("update_rate")) {
      rateHz = _sdf->Get<double>("update_rate");
    }
    if (_sdf->HasElement("frame_id")) {
      frame = _sdf->Get<std::string>("frame_id");
    }
    // Optional <model_name>...</model_name> entries: publish ground-truth pose
    // of these named models on the same topic (e.g. the robot).
    for (auto el = _sdf->FindElement("model_name");
         el != nullptr;
         el = el->GetNextElement("model_name"))
    {
      this->modelNames.push_back(el->Get<std::string>());
    }

    this->frameId = frame;
    this->updatePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rateHz));

    this->publisher = this->node.Advertise<gz::msgs::Pose_V>(topic);
    if (!this->publisher) {
      gzerr << "[ActorPosePublisher] failed to advertise topic [" << topic << "]\n";
      return;
    }
    gzmsg << "[ActorPosePublisher] publishing on [" << topic
          << "] at " << rateHz << " Hz (frame=" << frame
          << ", extra models=" << this->modelNames.size() << ")\n";
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused) {
      return;
    }
    if (_info.simTime - this->lastPublish < this->updatePeriod) {
      return;
    }
    this->lastPublish = _info.simTime;

    gz::msgs::Pose_V msg;
    auto * msgHeader = msg.mutable_header();
    const auto stampSec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    const auto stampNsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - stampSec);
    msgHeader->mutable_stamp()->set_sec(stampSec.count());
    msgHeader->mutable_stamp()->set_nsec(static_cast<int32_t>(stampNsec.count()));

    bool anyEntry = false;

    auto appendPose = [&](const std::string & _name,
                          uint32_t _id,
                          const gz::math::Pose3d & _pose)
    {
      auto * pose = msg.add_pose();
      pose->set_name(_name);
      pose->set_id(_id);

      auto * header = pose->mutable_header();
      header->mutable_stamp()->set_sec(stampSec.count());
      header->mutable_stamp()->set_nsec(static_cast<int32_t>(stampNsec.count()));
      auto * frameData = header->add_data();
      frameData->set_key("frame_id");
      frameData->add_value(this->frameId);
      auto * childData = header->add_data();
      childData->set_key("child_frame_id");
      childData->add_value(_name);

      pose->mutable_position()->set_x(_pose.Pos().X());
      pose->mutable_position()->set_y(_pose.Pos().Y());
      pose->mutable_position()->set_z(_pose.Pos().Z());
      pose->mutable_orientation()->set_x(_pose.Rot().X());
      pose->mutable_orientation()->set_y(_pose.Rot().Y());
      pose->mutable_orientation()->set_z(_pose.Rot().Z());
      pose->mutable_orientation()->set_w(_pose.Rot().W());
    };

    // 1) All actors (auto-discovered every step so newly spawned actors are picked up)
    _ecm.Each<gz::sim::components::Actor, gz::sim::components::Name>(
      [&](const gz::sim::Entity & _entity,
          const gz::sim::components::Actor *,
          const gz::sim::components::Name * _name) -> bool
      {
        gz::sim::Actor actor(_entity);
        auto worldPose = actor.WorldPose(_ecm);
        if (!worldPose.has_value()) {
          return true;
        }
        appendPose(_name->Data(), static_cast<uint32_t>(_entity), *worldPose);
        anyEntry = true;
        return true;
      });

    // 2) Configured model names (e.g. the robot). Cache entity lookups.
    for (const auto & name : this->modelNames) {
      auto cached = this->modelEntities.find(name);
      gz::sim::Entity entity = gz::sim::kNullEntity;
      if (cached != this->modelEntities.end()) {
        entity = cached->second;
      } else {
        _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity & _ent,
              const gz::sim::components::Model *,
              const gz::sim::components::Name * _n) -> bool
          {
            if (_n->Data() == name) {
              entity = _ent;
              return false;
            }
            return true;
          });
        if (entity != gz::sim::kNullEntity) {
          this->modelEntities[name] = entity;
        }
      }
      if (entity == gz::sim::kNullEntity) {
        continue;
      }
      const auto pose = gz::sim::worldPose(entity, _ecm);
      appendPose(name, static_cast<uint32_t>(entity), pose);
      anyEntry = true;
    }

    if (anyEntry) {
      this->publisher.Publish(msg);
    }
  }

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::chrono::steady_clock::duration updatePeriod{std::chrono::milliseconds(20)};
  std::chrono::steady_clock::duration lastPublish{std::chrono::seconds(0)};
  std::string frameId{"world"};
  std::vector<std::string> modelNames;
  std::unordered_map<std::string, gz::sim::Entity> modelEntities;
};

}  // namespace go2_simulation

GZ_ADD_PLUGIN(
  go2_simulation::ActorPosePublisher,
  gz::sim::System,
  go2_simulation::ActorPosePublisher::ISystemConfigure,
  go2_simulation::ActorPosePublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(go2_simulation::ActorPosePublisher,
  "go2_simulation::ActorPosePublisher")
