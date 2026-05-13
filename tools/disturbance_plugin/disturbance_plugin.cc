// disturbance_plugin.cc
//
// Gazebo Classic 11 ModelPlugin:
//   1) Applies the most-recent received Wrench to <link_name> every physics
//      step (force + torque + force_offset, all in body frame).
//   2) Optionally spawns a red cylinder "disturbance_arrow" model when force
//      is active, updates its world pose every step so it stays attached to
//      the drone, and despawns it when force returns to zero.
//
// The arrow is spawned as a SEPARATE static model via ~/factory (not as a
// child link of the host model) and its pose is driven by Model::SetWorldPose.
// This avoids the Gazebo Classic Visual-msg lookup quirk where ModelPlugin
// publications to ~/visual are not consistently applied by gzclient.
//
// SDF parameters:
//   <link_name>           Optional. Default "base_link".
//   <topic_name>          Optional. Default
//                         "/gazebo/<world>/<model>/disturbance_cmd".
//   <enable_arrow>        Optional. true|false. Default true.
//   <arrow_scale_factor>  Optional. Arrow length per N of force [m/N]. Default 0.05.
//   <arrow_max_length>    Optional. Cap arrow length [m]. Default 2.0.
//   <arrow_radius>        Optional. Cylinder radius [m]. Default 0.03.

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

#include <algorithm>
#include <mutex>
#include <sstream>
#include <string>

namespace gazebo {

class DisturbancePlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;

    const std::string linkName = _sdf->HasElement("link_name")
        ? _sdf->Get<std::string>("link_name")
        : std::string("base_link");

    this->link = this->model->GetLink(linkName);
    if (!this->link) {
      gzerr << "[DisturbancePlugin] link '" << linkName
            << "' not found in model '" << this->model->GetName() << "'\n";
      return;
    }

    const std::string worldName = this->model->GetWorld()->Name();
    const std::string defaultTopic =
        "/gazebo/" + worldName + "/" + this->model->GetName() + "/disturbance_cmd";
    const std::string topicName = _sdf->HasElement("topic_name")
        ? _sdf->Get<std::string>("topic_name")
        : defaultTopic;

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->sub = this->node->Subscribe(
        topicName, &DisturbancePlugin::OnWrench, this);

    // Arrow options
    if (_sdf->HasElement("enable_arrow")) {
      this->arrowEnabled = _sdf->Get<bool>("enable_arrow");
    }
    if (_sdf->HasElement("arrow_scale_factor")) {
      this->arrowScaleFactor = _sdf->Get<double>("arrow_scale_factor");
    }
    if (_sdf->HasElement("arrow_max_length")) {
      this->arrowMaxLength = _sdf->Get<double>("arrow_max_length");
    }
    if (_sdf->HasElement("arrow_radius")) {
      this->arrowRadius = _sdf->Get<double>("arrow_radius");
    }

    if (this->arrowEnabled) {
      this->arrowName = this->model->GetName() + "_disturbance_arrow";
      this->factoryPub = this->node->Advertise<gazebo::msgs::Factory>("~/factory");
      this->requestPub = this->node->Advertise<gazebo::msgs::Request>("~/request");
      gzmsg << "[DisturbancePlugin] arrow enabled: name=" << this->arrowName
            << ", scale=" << this->arrowScaleFactor << " m/N"
            << ", max=" << this->arrowMaxLength << " m"
            << ", radius=" << this->arrowRadius << " m\n";
    }

    this->updateConn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DisturbancePlugin::OnUpdate, this));

    gzmsg << "[DisturbancePlugin] loaded: model=" << this->model->GetName()
          << ", link=" << linkName
          << ", topic=" << topicName << "\n";
  }

  ~DisturbancePlugin() override {
    // Best-effort cleanup
    if (this->arrowSpawned && this->requestPub) {
      this->DespawnArrow();
    }
  }

 private:
  void OnWrench(ConstWrenchPtr &_msg) {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->force.Set(_msg->force().x(), _msg->force().y(), _msg->force().z());
    this->torque.Set(_msg->torque().x(), _msg->torque().y(), _msg->torque().z());
    if (_msg->has_force_offset()) {
      this->offset.Set(_msg->force_offset().x(),
                       _msg->force_offset().y(),
                       _msg->force_offset().z());
    } else {
      this->offset = ignition::math::Vector3d::Zero;
    }
    this->wrenchChanged = true;
  }

  void OnUpdate() {
    ignition::math::Vector3d cur_force, cur_torque, cur_offset;
    bool changed;
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      cur_force = this->force;
      cur_torque = this->torque;
      cur_offset = this->offset;
      changed = this->wrenchChanged;
      this->wrenchChanged = false;
    }

    // Apply force/torque (body frame)
    if (cur_force != ignition::math::Vector3d::Zero) {
      this->link->AddLinkForce(cur_force, cur_offset);
    }
    if (cur_torque != ignition::math::Vector3d::Zero) {
      this->link->AddRelativeTorque(cur_torque);
    }

    // Arrow lifecycle
    if (this->arrowEnabled) {
      const double mag = cur_force.Length();
      const bool shouldShow = (mag > 1e-3);

      if (changed) {
        // Wrench just changed: respawn arrow with new length, or despawn.
        if (this->arrowSpawned) {
          this->DespawnArrow();
          this->arrowSpawned = false;
        }
        if (shouldShow) {
          this->SpawnArrow(mag);
          this->arrowSpawned = true;
        }
      }

      // Update pose every step so arrow tracks the drone.
      if (this->arrowSpawned) {
        this->UpdateArrowPose(cur_force, cur_offset);
      }
    }
  }

  void SpawnArrow(double mag) {
    const double length =
        std::min(mag * this->arrowScaleFactor, this->arrowMaxLength);

    // 화살표 구조:
    //   - shaft: cylinder, total 길이의 80%, 반경 = arrowRadius
    //   - head : 단위 cone mesh (model://disturbance_arrow_assets/meshes/cone.stl)
    //            를 base 반경 = 3 * arrowRadius, 높이 = 길이의 20% 로 scale
    const double shaftLen = length * 0.8;
    const double headLen = length * 0.2;
    const double shaftR = this->arrowRadius;
    const double headR = this->arrowRadius * 3.0;

    std::ostringstream sdf;
    sdf << "<?xml version='1.0'?>\n"
        << "<sdf version='1.5'>\n"
        << "  <model name='" << this->arrowName << "'>\n"
        << "    <static>true</static>\n"
        << "    <link name='link'>\n"
        // shaft: link 원점에서 시작해 +Z로 shaftLen 까지 뻗음
        << "      <visual name='shaft'>\n"
        << "        <pose>0 0 " << (shaftLen / 2.0) << " 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <cylinder>\n"
        << "            <radius>" << shaftR << "</radius>\n"
        << "            <length>" << shaftLen << "</length>\n"
        << "          </cylinder>\n"
        << "        </geometry>\n"
        << "        <material>\n"
        << "          <ambient>1 0 0 1</ambient>\n"
        << "          <diffuse>1 0 0 1</diffuse>\n"
        << "          <emissive>0.7 0 0 1</emissive>\n"
        << "        </material>\n"
        << "      </visual>\n"
        // head: cone mesh, base 가 shaftLen 위치에 놓이도록.
        // 단위 cone STL은 base z=0, tip z=1. <scale> 로 x,y(반경) 와 z(높이) 조정.
        << "      <visual name='head'>\n"
        << "        <pose>0 0 " << shaftLen << " 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <mesh>\n"
        << "            <uri>model://disturbance_arrow_assets/meshes/cone.stl</uri>\n"
        << "            <scale>" << headR << " " << headR << " " << headLen << "</scale>\n"
        << "          </mesh>\n"
        << "        </geometry>\n"
        << "        <material>\n"
        << "          <ambient>1 0 0 1</ambient>\n"
        << "          <diffuse>1 0 0 1</diffuse>\n"
        << "          <emissive>0.7 0 0 1</emissive>\n"
        << "        </material>\n"
        << "      </visual>\n"
        << "    </link>\n"
        << "  </model>\n"
        << "</sdf>\n";

    gazebo::msgs::Factory msg;
    msg.set_sdf(sdf.str());
    this->factoryPub->Publish(msg);
  }

  void DespawnArrow() {
    gazebo::msgs::Request *req =
        gazebo::msgs::CreateRequest("entity_delete", this->arrowName);
    this->requestPub->Publish(*req);
    delete req;
  }

  void UpdateArrowPose(const ignition::math::Vector3d &cur_force,
                       const ignition::math::Vector3d &cur_offset) {
    auto world = this->model->GetWorld();
    auto arrowModel = world->ModelByName(this->arrowName);
    if (!arrowModel) return;  // factory spawn은 비동기 — 도착 전엔 null

    const auto droneWorldPose = this->link->WorldPose();

    // force는 body frame → world로 변환해 화살표 방향에 사용.
    const auto force_world = droneWorldPose.Rot().RotateVector(cur_force);
    const auto direction = force_world.Normalized();

    // Cylinder의 long axis는 Z. Z를 force 방향에 정렬.
    ignition::math::Quaterniond q;
    q.From2Axes(ignition::math::Vector3d::UnitZ, direction);

    // 화살표 base 위치 = drone link pos + (body-offset을 world로 회전한 값)
    const auto offset_world = droneWorldPose.Rot().RotateVector(cur_offset);
    const auto arrowPos = droneWorldPose.Pos() + offset_world;

    arrowModel->SetWorldPose(ignition::math::Pose3d(arrowPos, q));
  }

  // Physics handles
  physics::ModelPtr model;
  physics::LinkPtr link;

  // Transport
  transport::NodePtr node;
  transport::SubscriberPtr sub;
  transport::PublisherPtr factoryPub;
  transport::PublisherPtr requestPub;
  event::ConnectionPtr updateConn;

  // Shared state (mtx-protected)
  std::mutex mtx;
  ignition::math::Vector3d force{0, 0, 0};
  ignition::math::Vector3d torque{0, 0, 0};
  ignition::math::Vector3d offset{0, 0, 0};
  bool wrenchChanged = false;

  // Arrow state (only accessed inside OnUpdate after locking)
  bool arrowEnabled = true;
  std::string arrowName;
  bool arrowSpawned = false;
  double arrowScaleFactor = 0.05;
  double arrowMaxLength = 2.0;
  double arrowRadius = 0.03;
};

GZ_REGISTER_MODEL_PLUGIN(DisturbancePlugin)

}  // namespace gazebo
