/*
 * robin_sim_plugin — Gazebo Harmonic System plugin for ROBIN welding sim.
 *
 * Features:
 *   1. Progressive bead deposition: grows a live bead during welding,
 *      finalises on weld-end.
 *   2. Clear-beads command: removes all spawned bead entities.
 *
 * Touch sensing and wire feed are handled entirely by the FroniusSimulator
 * Python class (no Gazebo physics needed).
 */

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>

#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>

#include "robin_sim_gz/bead_model.hh"

#include <mutex>
#include <string>
#include <sstream>
#include <cmath>
#include <map>
#include <vector>

namespace robin_sim_gz
{

class RobinSimPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  // ── ISystemConfigure ────────────────────────────────────────────────
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm))
    {
      gzerr << "RobinSimPlugin: attached entity is not a model.\n";
      return;
    }

    this->eventMgr_ = &_eventMgr;
    auto modelName = this->model_.Name(_ecm);
    gzmsg << "RobinSimPlugin: loaded on model [" << modelName << "]\n";

    // ── Find wire_tip link (for bead deposition pose) ──
    this->wireTipEntity_ = _ecm.EntityByComponents(
      gz::sim::components::Name("wire_tip"),
      gz::sim::components::Link());
    if (this->wireTipEntity_ == gz::sim::kNullEntity)
      gzwarn << "RobinSimPlugin: wire_tip link not found — bead deposition disabled.\n";
    else
      gzmsg << "RobinSimPlugin: wire_tip link found.\n";

    // ── Read SDF parameters ──
    std::string weldActiveTopic = "/robin/weld_active";

    if (_sdf->HasElement("weld_active_topic"))
      weldActiveTopic = _sdf->Get<std::string>("weld_active_topic");

    // ── gz.transport subscriptions ──

    this->node_.Subscribe(weldActiveTopic,
      &RobinSimPlugin::OnWeldActive, this);
    gzmsg << "RobinSimPlugin: subscribed to [" << weldActiveTopic << "]\n";

    this->node_.Subscribe("/robin/weld_current",
      &RobinSimPlugin::OnWeldCurrent, this);
    this->node_.Subscribe("/robin/weld_wfs",
      &RobinSimPlugin::OnWeldWfs, this);
    this->node_.Subscribe("/robin/weld_voltage",
      &RobinSimPlugin::OnWeldVoltage, this);
    this->node_.Subscribe("/robin/weld_travel_speed",
      &RobinSimPlugin::OnWeldTravelSpeed, this);
    this->node_.Subscribe("/robin/weld_bead_id",
      &RobinSimPlugin::OnWeldBeadId, this);
    this->node_.Subscribe("/robin/clear_beads",
      &RobinSimPlugin::OnClearBeads, this);
    gzmsg << "RobinSimPlugin: subscribed to weld param + control topics\n";

    // Store the world entity for SdfEntityCreator
    auto worldEntity = _ecm.EntityByComponents(
      gz::sim::components::Name("welding_cell"));
    if (worldEntity == gz::sim::kNullEntity)
    {
      _ecm.Each<gz::sim::components::Name>(
        [&](const gz::sim::Entity & entity,
            const gz::sim::components::Name *) -> bool
        {
          if (!_ecm.Component<gz::sim::components::ParentEntity>(entity))
          {
            worldEntity = entity;
            return false;
          }
          return true;
        });
    }
    this->worldEntity_ = worldEntity;
  }

  // ── ISystemPreUpdate: track weld path + spawn beads ─────────────────
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused)
      return;

    // ── Handle clear-beads request ──
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->clearBeadsRequested_)
      {
        this->clearBeadsRequested_ = false;
        ClearAllBeads(_ecm);
      }
    }

    // ── Bead deposition (track during weld, spawn progressively) ──
    bool welding;
    double current, wfs, travelSpeed;
    std::string beadId;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      welding = this->weldActive_;
      current = this->weldCurrent_;
      wfs = this->weldWfs_;
      travelSpeed = this->weldTravelSpeed_;
      beadId = this->weldBeadId_;
    }

    if (welding && this->wireTipEntity_ != gz::sim::kNullEntity)
    {
      auto tipPose = gz::sim::worldPose(this->wireTipEntity_, _ecm);

      if (!this->weldTracking_)
      {
        // Weld just started — record start
        this->weldTracking_ = true;
        this->beadStartPos_ = tipPose.Pos();
        this->beadEndPos_ = tipPose.Pos();
        this->beadPathLength_ = 0.0;
        this->accumCurrent_ = current;
        this->accumWfs_ = wfs;
        this->accumTravelSpeed_ = travelSpeed;
        this->accumSamples_ = 1;
        this->liveBeadEntity_ = gz::sim::kNullEntity;
        this->liveUpdateCounter_ = 0;
        this->activeBeadId_ = beadId.empty()
          ? ("bead_" + std::to_string(this->beadCount_))
          : beadId;
      }
      else
      {
        // Accumulate path length + running average of weld params
        double step = tipPose.Pos().Distance(this->beadEndPos_);
        this->beadPathLength_ += step;
        this->beadEndPos_ = tipPose.Pos();
        this->accumCurrent_ += current;
        this->accumWfs_ += wfs;
        this->accumTravelSpeed_ += travelSpeed;
        this->accumSamples_++;

        // Progressive update every ~0.5s (200 ticks at 400 Hz)
        this->liveUpdateCounter_++;
        if (this->liveUpdateCounter_ >= 200 && this->beadPathLength_ > 0.001)
        {
          this->liveUpdateCounter_ = 0;
          UpdateLiveBead(_ecm);
        }
      }
    }
    else if (this->weldTracking_)
    {
      // Weld just ended — finalise the bead
      this->weldTracking_ = false;

      // Remove the live preview entity
      RemoveLiveBead(_ecm);

      if (this->beadPathLength_ > 0.001)  // skip trivial beads < 1mm
      {
        double n = static_cast<double>(this->accumSamples_);
        double avgCurrent = this->accumCurrent_ / n;
        double avgWfs = this->accumWfs_ / n;
        double avgTravelSpeed = this->accumTravelSpeed_ / n;

        auto section = bead_from_params(avgCurrent, avgWfs, avgTravelSpeed);

        auto [center, ori] = ComputeBeadPose(
          this->beadStartPos_, this->beadEndPos_);

        // Remove any previous bead with the same name
        auto oldIt = this->finalisedBeads_.find(this->activeBeadId_);
        if (oldIt != this->finalisedBeads_.end())
        {
          gz::sim::SdfEntityCreator creator(_ecm, *this->eventMgr_);
          creator.RequestRemoveEntity(oldIt->second);
          this->finalisedBeads_.erase(oldIt);
          gzmsg << "RobinSimPlugin: removed old bead [" << this->activeBeadId_ << "]\n";
        }

        auto entity = SpawnBeadEntity(
          _ecm, this->activeBeadId_, center, ori,
          this->beadPathLength_, section);

        if (entity != gz::sim::kNullEntity)
          this->finalisedBeads_[this->activeBeadId_] = entity;

        this->beadCount_++;

        gzmsg << "RobinSimPlugin: finalised [" << this->activeBeadId_
               << "] length=" << this->beadPathLength_ * 1000.0 << "mm"
               << " width=" << section.width * 1000.0 << "mm"
               << " height=" << section.height * 1000.0 << "mm"
               << " toe=" << section.toe_angle_rad * 180.0 / M_PI << "deg\n";
      }
    }
  }

private:
  // ── gz.transport callbacks ──

  void OnWeldActive(const gz::msgs::Boolean & _msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->weldActive_ = _msg.data();
  }

  void OnWeldCurrent(const gz::msgs::Double & m) {
    std::lock_guard<std::mutex> l(mutex_); weldCurrent_ = m.data(); }
  void OnWeldWfs(const gz::msgs::Double & m) {
    std::lock_guard<std::mutex> l(mutex_); weldWfs_ = m.data(); }
  void OnWeldVoltage(const gz::msgs::Double & m) {
    std::lock_guard<std::mutex> l(mutex_); weldVoltage_ = m.data(); }
  void OnWeldTravelSpeed(const gz::msgs::Double & m) {
    std::lock_guard<std::mutex> l(mutex_); weldTravelSpeed_ = m.data(); }
  void OnWeldBeadId(const gz::msgs::StringMsg & m) {
    std::lock_guard<std::mutex> l(mutex_); weldBeadId_ = m.data(); }
  void OnClearBeads(const gz::msgs::Boolean & m) {
    if (m.data()) {
      std::lock_guard<std::mutex> l(mutex_); clearBeadsRequested_ = true;
    }
  }

  // ── Pose computation ──

  std::pair<gz::math::Vector3d, gz::math::Quaterniond>
  ComputeBeadPose(const gz::math::Vector3d & start,
                  const gz::math::Vector3d & end) const
  {
    auto center = (start + end) * 0.5;

    // Orientation: polyline cross-section in X-Y, extrusion along Z.
    auto dir = (end - start).Normalized();
    auto up = gz::math::Vector3d::UnitZ;
    auto right = up.Cross(dir).Normalized();
    if (right.Length() < 0.001)
      right = gz::math::Vector3d::UnitY.Cross(dir).Normalized();
    up = dir.Cross(right).Normalized();

    gz::math::Matrix3d rotMat;
    rotMat.SetCol(0, right);
    rotMat.SetCol(1, up);
    rotMat.SetCol(2, dir);
    return {center, gz::math::Quaterniond(rotMat)};
  }

  // ── Progressive bead update ──

  void UpdateLiveBead(gz::sim::EntityComponentManager & _ecm)
  {
    RemoveLiveBead(_ecm);

    double n = static_cast<double>(this->accumSamples_);
    auto section = bead_from_params(
      this->accumCurrent_ / n,
      this->accumWfs_ / n,
      this->accumTravelSpeed_ / n);

    auto [center, ori] = ComputeBeadPose(
      this->beadStartPos_, this->beadEndPos_);

    std::string liveName = this->activeBeadId_ + "_live";
    this->liveBeadEntity_ = SpawnBeadEntity(
      _ecm, liveName, center, ori, this->beadPathLength_, section);
  }

  void RemoveLiveBead(gz::sim::EntityComponentManager & _ecm)
  {
    if (this->liveBeadEntity_ != gz::sim::kNullEntity)
    {
      gz::sim::SdfEntityCreator creator(_ecm, *this->eventMgr_);
      creator.RequestRemoveEntity(this->liveBeadEntity_);
      this->liveBeadEntity_ = gz::sim::kNullEntity;
    }
  }

  // ── Clear all beads ──

  void ClearAllBeads(gz::sim::EntityComponentManager & _ecm)
  {
    gz::sim::SdfEntityCreator creator(_ecm, *this->eventMgr_);
    for (auto & [name, entity] : this->finalisedBeads_)
      creator.RequestRemoveEntity(entity);
    this->finalisedBeads_.clear();
    RemoveLiveBead(_ecm);
    gzmsg << "RobinSimPlugin: cleared all beads\n";
  }

  // ── Bead spawning ──

  gz::sim::Entity SpawnBeadEntity(
    gz::sim::EntityComponentManager & _ecm,
    const std::string & _name,
    const gz::math::Vector3d & _center,
    const gz::math::Quaterniond & _ori,
    double _length, const BeadSection & _section)
  {
    // Compute cross-section profile (cubic crown with toe angle)
    auto profile = bead_cross_section(_section, 16);

    // Build SDF <polyline> points
    std::ostringstream polyPts;
    for (auto & [px, py] : profile)
      polyPts << "        <point>" << px << " " << py << "</point>\n";

    std::ostringstream sdfStr;
    sdfStr <<
      "<?xml version='1.0'?>"
      "<sdf version='1.9'>"
      "<model name='" << _name << "'>"
      "  <static>true</static>"
      "  <pose>"
           << _center.X() << " " << _center.Y() << " " << _center.Z() << " "
           << _ori.Roll() << " " << _ori.Pitch() << " " << _ori.Yaw() <<
      "  </pose>"
      "  <link name='bead_link'>"
      "    <visual name='bead_vis'>"
      "      <pose>0 0 " << (-_length / 2.0) << " 0 0 0</pose>"
      "      <geometry>"
      "        <polyline>\n"
               << polyPts.str() <<
      "          <height>" << _length << "</height>"
      "        </polyline>"
      "      </geometry>"
      "      <material>"
      "        <ambient>0.45 0.43 0.40 1</ambient>"
      "        <diffuse>0.30 0.28 0.26 1</diffuse>"
      "        <specular>0.9 0.85 0.8 1</specular>"
      "      </material>"
      "    </visual>"
      "  </link>"
      "</model>"
      "</sdf>";

    sdf::Root root;
    auto errors = root.LoadSdfString(sdfStr.str());
    if (!errors.empty())
    {
      gzerr << "RobinSimPlugin: bead SDF parse error: "
             << errors.front().Message() << "\n";
      return gz::sim::kNullEntity;
    }

    auto * sdfModel = root.Model();
    if (!sdfModel)
    {
      gzerr << "RobinSimPlugin: bead SDF has no model\n";
      return gz::sim::kNullEntity;
    }

    gz::sim::SdfEntityCreator creator(_ecm, *this->eventMgr_);
    auto beadEntity = creator.CreateEntities(sdfModel);
    creator.SetParent(beadEntity, this->worldEntity_);
    return beadEntity;
  }

  // ── Member data ──

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity wireTipEntity_{gz::sim::kNullEntity};
  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
  gz::sim::EventManager * eventMgr_{nullptr};

  gz::transport::Node node_;

  std::mutex mutex_;

  // Welding state (written by gz.transport callbacks, read in PreUpdate)
  bool weldActive_{false};
  double weldCurrent_{0.0};
  double weldWfs_{0.0};
  double weldVoltage_{0.0};
  double weldTravelSpeed_{0.0};
  std::string weldBeadId_;
  bool clearBeadsRequested_{false};

  // Bead tracking (used in PreUpdate only — no lock needed)
  bool weldTracking_{false};
  gz::math::Vector3d beadStartPos_;
  gz::math::Vector3d beadEndPos_;
  double beadPathLength_{0.0};
  double accumCurrent_{0.0};
  double accumWfs_{0.0};
  double accumTravelSpeed_{0.0};
  uint32_t accumSamples_{0};
  uint32_t beadCount_{0};

  // Active bead ID (used in PreUpdate only)
  std::string activeBeadId_;

  // Live bead entity (progressive preview during welding)
  gz::sim::Entity liveBeadEntity_{gz::sim::kNullEntity};
  uint32_t liveUpdateCounter_{0};

  // All finalised bead entities keyed by bead name (for clear + collision)
  std::map<std::string, gz::sim::Entity> finalisedBeads_;
};

}  // namespace robin_sim_gz

GZ_ADD_PLUGIN(
  robin_sim_gz::RobinSimPlugin,
  gz::sim::System,
  robin_sim_gz::RobinSimPlugin::ISystemConfigure,
  robin_sim_gz::RobinSimPlugin::ISystemPreUpdate)