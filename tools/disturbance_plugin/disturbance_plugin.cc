// disturbance_plugin.cc
//
// Gazebo Classic 11 ModelPlugin — JSON-driven multi-disturbance injector.
//
// Receives a disturbance-profile JSON (as a GzString message on <topic_name>),
// then plays back a timeline of disturbances. Each disturbance:
//   - is an Ornstein-Uhlenbeck (band-limited) random force + torque with a
//     per-disturbance correlation time (tau / correlation_time),
//   - acts in either 'body' (FLU, attached to the airframe) or 'global'
//     (world ENU) frame — see README §7.6 for the convention,
//   - is active over [start_time, start_time + duration], measured from the
//     instant the JSON trigger arrived (t0),
//   - gets its own arrow visual showing the MEAN force direction.
//
// Multiple disturbances may overlap in time; their wrenches are summed by
// Gazebo within each physics step.
//
// OU exact discretization (per axis):
//   alpha = exp(-dt/tau),  beta = sqrt(1 - alpha^2)
//   F[k+1] = mean + (F[k] - mean) * alpha + stddev * beta * N(0,1)
//   -> as tau -> 0 this degenerates to white noise mean + stddev*N(0,1);
//      stable for all tau > 0; stddev is the steady-state std.
//
// NOTE on 'offset': interpreted in the BODY frame for both 'body' and 'global'
// disturbances, because an offset is physically a point on the airframe (e.g.
// a spray nozzle). The 'frame' field governs only the force/torque direction.
//
// SDF parameters:
//   <link_name>           Optional. Default "base_link".
//   <topic_name>          Optional. Default
//                         "/gazebo/<world>/<model>/disturbance_json".
//   <enable_arrow>        Optional. true|false. Default true.
//   <arrow_scale_factor>  Optional. Arrow length per N of mean force [m/N]. Default 0.05.
//   <arrow_max_length>    Optional. Cap arrow length [m]. Default 2.0.
//   <arrow_radius>        Optional. Shaft radius [m]. Default 0.03.

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>

#include <json/json.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace gazebo {

namespace {
// Color palette so overlapping disturbances stay visually distinguishable.
const std::array<std::array<double, 3>, 6> kArrowColors = {{
    {{1.00, 0.10, 0.10}}, {{0.15, 0.45, 1.00}}, {{0.15, 0.85, 0.20}},
    {{1.00, 0.65, 0.00}}, {{0.75, 0.20, 1.00}}, {{0.00, 0.85, 0.85}},
}};
}  // namespace

struct Disturbance {
  std::string name;
  bool global = false;  // true: world ENU frame, false: body FLU frame
  double startTime = 0.0;
  double duration = 0.0;
  double tau = 1.0;
  ignition::math::Vector3d forceMean, forceStddev;
  ignition::math::Vector3d torqueMean, torqueStddev;
  ignition::math::Vector3d offset;  // body frame, relative to link origin

  // runtime state
  bool active = false;
  ignition::math::Vector3d forceState, torqueState;  // OU states
  bool arrowSpawned = false;
  std::string arrowName;
  std::array<double, 3> color{{1.0, 0.1, 0.1}};
};

class DisturbancePlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;
    this->world = _model->GetWorld();

    const std::string linkName = _sdf->HasElement("link_name")
        ? _sdf->Get<std::string>("link_name")
        : std::string("base_link");
    this->link = this->model->GetLink(linkName);
    if (!this->link) {
      gzerr << "[DisturbancePlugin] link '" << linkName
            << "' not found in model '" << this->model->GetName() << "'\n";
      return;
    }

    const std::string worldName = this->world->Name();
    const std::string defaultTopic =
        "/gazebo/" + worldName + "/" + this->model->GetName() + "/disturbance_json";
    const std::string topicName = _sdf->HasElement("topic_name")
        ? _sdf->Get<std::string>("topic_name")
        : defaultTopic;

    if (_sdf->HasElement("enable_arrow"))
      this->arrowEnabled = _sdf->Get<bool>("enable_arrow");
    if (_sdf->HasElement("arrow_scale_factor"))
      this->arrowScaleFactor = _sdf->Get<double>("arrow_scale_factor");
    if (_sdf->HasElement("arrow_max_length"))
      this->arrowMaxLength = _sdf->Get<double>("arrow_max_length");
    if (_sdf->HasElement("arrow_radius"))
      this->arrowRadius = _sdf->Get<double>("arrow_radius");

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->sub = this->node->Subscribe(topicName, &DisturbancePlugin::OnJson, this);
    if (this->arrowEnabled) {
      this->factoryPub = this->node->Advertise<gazebo::msgs::Factory>("~/factory");
      this->requestPub = this->node->Advertise<gazebo::msgs::Request>("~/request");
    }

    this->rng.seed(std::random_device{}());

    this->updateConn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DisturbancePlugin::OnUpdate, this));

    gzmsg << "[DisturbancePlugin] loaded: model=" << this->model->GetName()
          << ", link=" << linkName << ", topic=" << topicName
          << ", arrow=" << (this->arrowEnabled ? "on" : "off") << "\n";
  }

  ~DisturbancePlugin() override {
    for (auto &d : this->disturbances)
      if (d.arrowSpawned) this->DespawnArrow(d.arrowName);
  }

 private:
  // --- transport callback (transport thread) ---
  void OnJson(ConstGzStringPtr &_msg) {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->pendingJson = _msg->data();
    this->hasPending = true;
  }

  // --- per physics-step (physics thread) ---
  void OnUpdate() {
    // Handle a pending trigger: parse + reset timeline.
    std::string js;
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      if (this->hasPending) {
        js = this->pendingJson;
        this->hasPending = false;
      }
    }
    if (!js.empty()) this->LoadProfile(js);

    if (!this->triggered) return;

    const double simTime = this->world->SimTime().Double();
    const double dt = simTime - this->lastSimTime;
    this->lastSimTime = simTime;
    if (dt <= 0.0 || dt > 1.0) return;  // trigger step / time jump — skip

    const double tRel = simTime - this->t0;
    const auto linkPose = this->link->WorldPose();

    for (auto &d : this->disturbances) {
      const bool shouldBeActive =
          (tRel >= d.startTime) && (tRel < d.startTime + d.duration);

      if (shouldBeActive && !d.active) {
        d.active = true;
        d.forceState = d.forceMean;     // start OU at the mean
        d.torqueState = d.torqueMean;
        if (this->arrowEnabled) this->TrySpawnArrow(d);
      } else if (!shouldBeActive && d.active) {
        d.active = false;
        if (d.arrowSpawned) {
          this->DespawnArrow(d.arrowName);
          d.arrowSpawned = false;
        }
      }

      if (!d.active) continue;

      d.forceState =
          this->OuStep(d.forceState, d.forceMean, d.forceStddev, d.tau, dt);
      d.torqueState =
          this->OuStep(d.torqueState, d.torqueMean, d.torqueStddev, d.tau, dt);

      if (d.global) {
        // force/torque in world ENU; offset is a body-frame point on the airframe
        const auto worldPos =
            linkPose.Pos() + linkPose.Rot().RotateVector(d.offset);
        this->link->AddForceAtWorldPosition(d.forceState, worldPos);
        this->link->AddTorque(d.torqueState);
      } else {
        // force/torque in body FLU
        this->link->AddLinkForce(d.forceState, d.offset);
        this->link->AddRelativeTorque(d.torqueState);
      }

      if (d.arrowSpawned) this->UpdateArrowPose(d, linkPose);
    }
  }

  // --- JSON profile parsing ---
  void LoadProfile(const std::string &js) {
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream iss(js);
    if (!Json::parseFromStream(builder, iss, &root, &errs)) {
      gzerr << "[DisturbancePlugin] JSON parse error: " << errs << "\n";
      return;
    }

    // Despawn arrows left over from a previous run.
    for (auto &d : this->disturbances)
      if (d.arrowSpawned) this->DespawnArrow(d.arrowName);
    this->disturbances.clear();

    const Json::Value &arr = root["disturbances"];
    if (!arr.isArray()) {
      gzerr << "[DisturbancePlugin] JSON has no 'disturbances' array\n";
      return;
    }

    for (Json::ArrayIndex i = 0; i < arr.size(); ++i) {
      const Json::Value &e = arr[i];
      Disturbance d;
      d.name = e.get("name", "dist_" + std::to_string(i)).asString();
      d.global = (e.get("frame", "body").asString() == "global");
      d.startTime = e.get("start_time", 0.0).asDouble();
      d.duration = e.get("duration", 0.0).asDouble();
      d.tau = e.get("correlation_time", 1.0).asDouble();
      d.forceMean = JsonVec3(e["force"]["mean"]);
      d.forceStddev = JsonVec3(e["force"]["stddev"]);
      d.torqueMean = JsonVec3(e["torque"]["mean"]);
      d.torqueStddev = JsonVec3(e["torque"]["stddev"]);
      d.offset = JsonVec3(e["offset"]);
      d.color = kArrowColors[i % kArrowColors.size()];
      d.arrowName = this->model->GetName() + "_dist_" + SanitizeName(d.name);
      this->disturbances.push_back(d);

      gzmsg << "[DisturbancePlugin]   [" << i << "] " << d.name
            << " frame=" << (d.global ? "global" : "body")
            << " t=[" << d.startTime << "," << (d.startTime + d.duration) << "]s"
            << " tau=" << d.tau << "\n";
    }

    this->t0 = this->world->SimTime().Double();
    this->lastSimTime = this->t0;
    this->triggered = true;
    gzmsg << "[DisturbancePlugin] profile loaded: " << this->disturbances.size()
          << " disturbance(s), t0=" << this->t0 << "s\n";
  }

  // --- helpers ---
  static ignition::math::Vector3d JsonVec3(const Json::Value &v) {
    if (!v.isArray() || v.size() < 3) return ignition::math::Vector3d::Zero;
    return ignition::math::Vector3d(
        v[0].asDouble(), v[1].asDouble(), v[2].asDouble());
  }

  static std::string SanitizeName(const std::string &s) {
    std::string out;
    for (char c : s)
      out += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
    return out.empty() ? std::string("x") : out;
  }

  ignition::math::Vector3d OuStep(const ignition::math::Vector3d &state,
                                  const ignition::math::Vector3d &mean,
                                  const ignition::math::Vector3d &stddev,
                                  double tau, double dt) {
    const double alpha = (tau > 1e-6) ? std::exp(-dt / tau) : 0.0;
    const double beta = std::sqrt(std::max(0.0, 1.0 - alpha * alpha));
    return ignition::math::Vector3d(
        mean.X() + (state.X() - mean.X()) * alpha
            + stddev.X() * beta * this->gauss(this->rng),
        mean.Y() + (state.Y() - mean.Y()) * alpha
            + stddev.Y() * beta * this->gauss(this->rng),
        mean.Z() + (state.Z() - mean.Z()) * alpha
            + stddev.Z() * beta * this->gauss(this->rng));
  }

  // --- arrow visualization (one model per disturbance) ---
  void TrySpawnArrow(Disturbance &d) {
    const double meanMag = d.forceMean.Length();
    if (meanMag < 1e-3) return;  // no meaningful mean direction — skip arrow
    const double length =
        std::min(meanMag * this->arrowScaleFactor, this->arrowMaxLength);
    this->SpawnArrow(d.arrowName, length, d.color);
    d.arrowSpawned = true;
  }

  void SpawnArrow(const std::string &name, double length,
                  const std::array<double, 3> &color) {
    const double shaftLen = length * 0.8;
    const double headLen = length * 0.2;
    const double shaftR = this->arrowRadius;
    const double headR = this->arrowRadius * 3.0;

    std::ostringstream c, em;
    c << color[0] << " " << color[1] << " " << color[2] << " 1";
    em << color[0] * 0.6 << " " << color[1] * 0.6 << " " << color[2] * 0.6 << " 1";

    std::ostringstream sdf;
    sdf << "<?xml version='1.0'?>\n<sdf version='1.5'>\n"
        << "  <model name='" << name << "'>\n    <static>true</static>\n"
        << "    <link name='link'>\n"
        // shaft: cylinder, link origin -> +Z, 80% of total length
        << "      <visual name='shaft'>\n"
        << "        <pose>0 0 " << (shaftLen / 2.0) << " 0 0 0</pose>\n"
        << "        <geometry><cylinder><radius>" << shaftR
        << "</radius><length>" << shaftLen << "</length></cylinder></geometry>\n"
        << "        <material><ambient>" << c.str() << "</ambient><diffuse>"
        << c.str() << "</diffuse><emissive>" << em.str()
        << "</emissive></material>\n"
        << "      </visual>\n"
        // head: unit cone STL (base z=0, tip z=1), scaled to radius/height
        << "      <visual name='head'>\n"
        << "        <pose>0 0 " << shaftLen << " 0 0 0</pose>\n"
        << "        <geometry><mesh>"
        << "<uri>model://disturbance_arrow_assets/meshes/cone.stl</uri>"
        << "<scale>" << headR << " " << headR << " " << headLen << "</scale>"
        << "</mesh></geometry>\n"
        << "        <material><ambient>" << c.str() << "</ambient><diffuse>"
        << c.str() << "</diffuse><emissive>" << em.str()
        << "</emissive></material>\n"
        << "      </visual>\n    </link>\n  </model>\n</sdf>\n";

    gazebo::msgs::Factory msg;
    msg.set_sdf(sdf.str());
    this->factoryPub->Publish(msg);
  }

  void DespawnArrow(const std::string &name) {
    if (!this->requestPub) return;
    gazebo::msgs::Request *req =
        gazebo::msgs::CreateRequest("entity_delete", name);
    this->requestPub->Publish(*req);
    delete req;
  }

  void UpdateArrowPose(const Disturbance &d,
                       const ignition::math::Pose3d &linkPose) {
    auto arrowModel = this->world->ModelByName(d.arrowName);
    if (!arrowModel) return;  // factory spawn is async — not here yet

    // Arrow shows the MEAN force direction (constant per disturbance).
    ignition::math::Vector3d dirWorld =
        d.global ? d.forceMean : linkPose.Rot().RotateVector(d.forceMean);
    if (dirWorld.Length() < 1e-6) return;
    dirWorld.Normalize();

    ignition::math::Quaterniond q;
    q.From2Axes(ignition::math::Vector3d::UnitZ, dirWorld);

    const auto arrowPos =
        linkPose.Pos() + linkPose.Rot().RotateVector(d.offset);
    arrowModel->SetWorldPose(ignition::math::Pose3d(arrowPos, q));
  }

  // --- physics handles ---
  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::LinkPtr link;

  // --- transport ---
  transport::NodePtr node;
  transport::SubscriberPtr sub;
  transport::PublisherPtr factoryPub;
  transport::PublisherPtr requestPub;
  event::ConnectionPtr updateConn;

  // --- trigger handoff (mtx-protected) ---
  std::mutex mtx;
  std::string pendingJson;
  bool hasPending = false;

  // --- timeline state (physics thread only) ---
  std::vector<Disturbance> disturbances;
  bool triggered = false;
  double t0 = 0.0;
  double lastSimTime = 0.0;

  // --- RNG (physics thread only) ---
  std::mt19937 rng;
  std::normal_distribution<double> gauss{0.0, 1.0};

  // --- arrow options ---
  bool arrowEnabled = true;
  double arrowScaleFactor = 0.05;
  double arrowMaxLength = 2.0;
  double arrowRadius = 0.03;
};

GZ_REGISTER_MODEL_PLUGIN(DisturbancePlugin)

}  // namespace gazebo
