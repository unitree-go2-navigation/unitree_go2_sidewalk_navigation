// Model-scoped gz-sim system plugin: moves its model kinematically along
// [t x y yaw] waypoints (sim-time linear interpolation) via WorldPoseCmd.
//
// gz <actor>의 대체물. 근거(2026-07-09 회귀 디버깅): actor는 스킨 유무와
// 무관하게 headless(-s) 서버측 센서 렌더링 초기화 시점에 sim 루프를
// wedge시킨다. 일반 model은 센서 씬에서 정상 렌더링됨(static_stop 검증).
// 모델에 collision을 두지 않으면 actor처럼 물리 통과(pass-through)한다.

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace go2_simulation
{

class WaypointMover
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm)) {
      gzerr << "[WaypointMover] must be attached to a <model>\n";
      return;
    }

    this->z = _sdf->Get<double>("z", 1.01).first;
    this->loop = _sdf->Get<bool>("loop", false).first;

    // <waypoint>t x y yaw</waypoint> (t는 단조증가, 최소 2개)
    for (auto el = _sdf->FindElement("waypoint");
         el != nullptr;
         el = el->GetNextElement("waypoint"))
    {
      std::istringstream ss(el->Get<std::string>());
      Waypoint w{};
      if (!(ss >> w.t >> w.x >> w.y >> w.yaw)) {
        gzerr << "[WaypointMover] bad <waypoint> (want: t x y yaw): "
              << el->Get<std::string>() << "\n";
        this->waypoints.clear();
        return;
      }
      this->waypoints.push_back(w);
    }
    if (this->waypoints.size() < 2) {
      gzerr << "[WaypointMover] needs >= 2 waypoints, got "
            << this->waypoints.size() << "\n";
      return;
    }
    gzmsg << "[WaypointMover] model [" << this->model.Name(_ecm) << "] "
          << this->waypoints.size() << " waypoints, loop="
          << (this->loop ? "true" : "false") << "\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused || this->waypoints.size() < 2) {
      return;
    }

    double t = std::chrono::duration<double>(_info.simTime).count();
    const double t0 = this->waypoints.front().t;
    const double t1 = this->waypoints.back().t;
    if (this->loop && t > t1) {
      t = t0 + std::fmod(t - t0, t1 - t0);
    }

    Waypoint p;
    if (t <= t0) {
      p = this->waypoints.front();
    } else if (t >= t1) {
      p = this->waypoints.back();  // loop=false: 마지막 지점에서 정지 유지
    } else {
      size_t i = 1;
      while (this->waypoints[i].t < t) {
        ++i;
      }
      const Waypoint & a = this->waypoints[i - 1];
      const Waypoint & b = this->waypoints[i];
      const double s = (t - a.t) / (b.t - a.t);
      p.x = a.x + s * (b.x - a.x);
      p.y = a.y + s * (b.y - a.y);
      // 최단호 yaw 보간
      double dyaw = std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
      p.yaw = a.yaw + s * dyaw;
    }

    this->model.SetWorldPoseCmd(
      _ecm, gz::math::Pose3d(p.x, p.y, this->z, 0.0, 0.0, p.yaw));
  }

private:
  struct Waypoint
  {
    double t, x, y, yaw;
  };

  gz::sim::Model model{gz::sim::kNullEntity};
  std::vector<Waypoint> waypoints;
  double z{1.01};
  bool loop{false};
};

}  // namespace go2_simulation

GZ_ADD_PLUGIN(
  go2_simulation::WaypointMover,
  gz::sim::System,
  go2_simulation::WaypointMover::ISystemConfigure,
  go2_simulation::WaypointMover::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(go2_simulation::WaypointMover,
  "go2_simulation::WaypointMover")
