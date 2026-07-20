/*
 * Copyright (C) 2024 ardupilot.org
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "BetaFlightPlugin.hh"

#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/imu.pb.h>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "SocketUDP.hh"
#include "Util.hh"

// Betaflight SITL wire format. These MUST stay byte-compatible with
// betaflight/src/platform/SIMULATOR/target/SITL/target.h.
namespace
{
/// \brief State sent Gazebo -> Betaflight (UDP port 9003).
struct fdm_packet
{
  double timestamp;                       // seconds
  double imu_angular_velocity_rpy[3];     // rad/s, body frame (FRD)
  double imu_linear_acceleration_xyz[3];  // m/s^2, body frame (FRD)
  double imu_orientation_quat[4];         // w, x, y, z
  double velocity_xyz[3];                 // m/s, earth frame (NED)
  double position_xyz[3];                 // m, NED from origin
  double pressure;                        // Pa
};

/// \brief Motor commands sent Betaflight -> Gazebo (UDP port 9002).
struct servo_packet
{
  float motor_speed[4];  // normalised [0.0, 1.0]
};
}  // namespace

// Register plugin.
GZ_ADD_PLUGIN(gz::sim::systems::BetaFlightPlugin,
              gz::sim::System,
              gz::sim::systems::BetaFlightPlugin::ISystemConfigure,
              gz::sim::systems::BetaFlightPlugin::ISystemPostUpdate,
              gz::sim::systems::BetaFlightPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::BetaFlightPlugin, "BetaFlightPlugin")

/// \brief A single rotor/motor control channel.
class BfControl
{
  /// \brief The motor_speed[] index this control reads.
  public: int channel = 0;

  /// \brief The name of the controlled joint.
  public: std::string jointName;

  /// \brief The controlled joint entity.
  public: gz::sim::Entity joint{gz::sim::kNullEntity};

  /// \brief Scales the [0,1] command to a target joint velocity (rad/s).
  /// The sign encodes the rotor spin direction (as in iris_with_standoffs).
  public: double multiplier = 838.0;

  /// \brief Offset applied to the raw [0,1] command.
  public: double offset = 0.0;

  /// \brief Slowdown factor for the simulated rotor velocity.
  public: double rotorVelocitySlowdownSim = 1.0;

  /// \brief The latest commanded target (rad/s).
  public: double cmd = 0.0;

  /// \brief Velocity PID used to drive the joint to the target velocity.
  public: gz::math::PID pid;
};

/// \brief Private data class.
class gz::sim::systems::BetaFlightPluginPrivate
{
  /// \brief The model.
  public: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief The model name.
  public: std::string modelName;

  /// \brief The world.
  public: gz::sim::World world{gz::sim::kNullEntity};

  /// \brief The world name.
  public: std::string worldName;

  /// \brief The rotor control channels.
  public: std::vector<BfControl> controls;

  /// \brief The link that carries the IMU sensor.
  public: gz::sim::Entity imuLink{gz::sim::kNullEntity};

  /// \brief The name of the IMU sensor.
  public: std::string imuName;

  /// \brief True once the IMU subscription has been set up.
  public: bool imuInitialized{false};

  /// \brief The most recently received IMU message.
  public: gz::msgs::IMU imuMsg;

  /// \brief True once at least one IMU message has been received.
  public: bool imuMsgValid{false};

  /// \brief Protects imuMsg / imuMsgValid.
  public: std::mutex imuMsgMutex;

  /// \brief Latch the latest IMU message.
  public: void ImuCb(const gz::msgs::IMU &_msg)
  {
    std::lock_guard<std::mutex> lock(this->imuMsgMutex);
    this->imuMsg = _msg;
    this->imuMsgValid = true;
  }

  /// \brief gz-transport node used for the IMU subscription.
  public: gz::transport::Node node;

  /// \brief UDP socket (bound to fdm_port_in, also used to send state).
  public: SocketUDP sock = SocketUDP(true, false);

  /// \brief Address Betaflight is reachable on.
  public: std::string fdm_address{"127.0.0.1"};

  /// \brief Local port that receives motor packets from Betaflight.
  public: uint16_t fdm_port_in{9002};

  /// \brief Betaflight port that receives the state packet.
  public: uint16_t fdm_port_out{9003};

  /// \brief True once a motor packet has been received from Betaflight.
  public: bool betaflightOnline{false};

  /// \brief Transform from model body frame to x-forward, z-down (FRD).
  public: gz::math::Pose3d modelXYZToAirplaneXForwardZDown;

  /// \brief Transform from the Gazebo world frame to NED.
  public: gz::math::Pose3d gazeboXYZToNED;

  /// \brief Time of the last controller update (sim time).
  public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Controller update mutex.
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
gz::sim::systems::BetaFlightPlugin::BetaFlightPlugin()
  : dataPtr(new BetaFlightPluginPrivate())
{
}

/////////////////////////////////////////////////
gz::sim::systems::BetaFlightPlugin::~BetaFlightPlugin()
{
}

/////////////////////////////////////////////////
void gz::sim::systems::BetaFlightPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  sdf::ElementPtr sdfClone = _sdf->Clone();

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "BetaFlightPlugin should be attached to a model entity. "
          << "Failed to initialize.\n";
    return;
  }
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(
      _ecm.EntityByComponents(components::World()));
  if (this->dataPtr->world.Name(_ecm).has_value())
  {
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();
  }

  // Frame conventions (identical to ArduPilotPlugin).
  this->dataPtr->modelXYZToAirplaneXForwardZDown =
      gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("modelXYZToAirplaneXForwardZDown"))
  {
    this->dataPtr->modelXYZToAirplaneXForwardZDown =
        sdfClone->Get<gz::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
  }

  this->dataPtr->gazeboXYZToNED = gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("gazeboXYZToNED"))
  {
    this->dataPtr->gazeboXYZToNED =
        sdfClone->Get<gz::math::Pose3d>("gazeboXYZToNED");
  }

  this->dataPtr->imuName =
      sdfClone->Get("imuName", static_cast<std::string>("imu_sensor")).first;

  this->LoadControlChannels(sdfClone, _ecm);

  if (!this->InitSockets(sdfClone))
  {
    return;
  }

  gzlog << "[" << this->dataPtr->modelName << "] "
        << "Betaflight ready to fly.\n";
}

/////////////////////////////////////////////////
void gz::sim::systems::BetaFlightPlugin::LoadControlChannels(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &_ecm)
{
  sdf::ElementPtr controlSDF;
  if (_sdf->HasElement("control"))
  {
    controlSDF = _sdf->GetElement("control");
  }

  while (controlSDF)
  {
    BfControl control;

    if (controlSDF->HasAttribute("channel"))
    {
      control.channel =
          atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
    }
    else
    {
      control.channel = this->dataPtr->controls.size();
    }

    if (controlSDF->HasElement("jointName"))
    {
      control.jointName = controlSDF->Get<std::string>("jointName");
    }
    else
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "control channel is missing a <jointName>.\n";
    }

    control.joint = JointByName(_ecm,
        this->dataPtr->model.Entity(), control.jointName);
    if (control.joint == gz::sim::kNullEntity)
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "couldn't find joint [" << control.jointName
            << "]. This plugin will not run.\n";
      return;
    }

    control.multiplier = controlSDF->Get("multiplier", control.multiplier).first;
    control.offset = controlSDF->Get("offset", control.offset).first;
    control.rotorVelocitySlowdownSim =
        controlSDF->Get("rotorVelocitySlowdownSim",
            control.rotorVelocitySlowdownSim).first;
    if (gz::math::equal(control.rotorVelocitySlowdownSim, 0.0))
    {
      control.rotorVelocitySlowdownSim = 1.0;
    }

    // Velocity PID (defaults match the iris rotor tuning).
    control.pid.Init(
        controlSDF->Get("p_gain", 0.20).first,
        controlSDF->Get("i_gain", 0.0).first,
        controlSDF->Get("d_gain", 0.0).first,
        controlSDF->Get("i_max", 0.0).first,
        controlSDF->Get("i_min", 0.0).first,
        controlSDF->Get("cmd_max", 2.5).first,
        controlSDF->Get("cmd_min", -2.5).first);
    control.pid.SetCmd(0.0);

    this->dataPtr->controls.push_back(control);
    controlSDF = controlSDF->GetNextElement("control");
  }
}

/////////////////////////////////////////////////
bool gz::sim::systems::BetaFlightPlugin::InitSockets(sdf::ElementPtr _sdf) const
{
  this->dataPtr->fdm_address =
      _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
  this->dataPtr->fdm_port_in =
      _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;
  this->dataPtr->fdm_port_out =
      _sdf->Get("fdm_port_out", static_cast<uint32_t>(9003)).first;

  if (!this->dataPtr->sock.bind(this->dataPtr->fdm_address.c_str(),
      this->dataPtr->fdm_port_in))
  {
    gzerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind " << this->dataPtr->fdm_address << ":"
          << this->dataPtr->fdm_port_in << ", aborting plugin.\n";
    return false;
  }
  gzlog << "[" << this->dataPtr->modelName << "] "
        << "Betaflight bridge: listening for motors on "
        << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in
        << ", sending state to " << this->dataPtr->fdm_address << ":"
        << this->dataPtr->fdm_port_out << "\n";
  return true;
}

/////////////////////////////////////////////////
void gz::sim::systems::BetaFlightPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  // Deferred to PreUpdate because the fully-qualified IMU topic name is not
  // available in Configure().
  if (!this->dataPtr->imuInitialized)
  {
    this->dataPtr->imuInitialized = true;

    auto entities = entitiesFromScopedName(
        this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());
    if (entities.empty())
    {
      entities = EntitiesFromUnscopedName(
          this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());
    }

    if (entities.empty())
    {
      gzerr << "[" << this->dataPtr->modelName << "] imu_sensor ["
            << this->dataPtr->imuName << "] not found, abort plugin.\n";
      return;
    }

    gz::sim::Entity imuEntity = *entities.begin();
    gz::sim::Entity parent = _ecm.ParentEntity(imuEntity);
    if (!_ecm.EntityHasComponentType(parent, components::Link::typeId))
    {
      gzerr << "[" << this->dataPtr->modelName << "] parent of IMU ["
            << this->dataPtr->imuName << "] is not a link.\n";
      return;
    }
    this->dataPtr->imuLink = parent;

    std::string imuTopicName = gz::sim::scopedName(imuEntity, _ecm) + "/imu";
    this->dataPtr->node.Subscribe(imuTopicName,
        &gz::sim::systems::BetaFlightPluginPrivate::ImuCb,
        this->dataPtr.get());

    // Components needed to build the state packet later.
    enableComponent<components::WorldPose>(
        _ecm, this->dataPtr->imuLink, true);
    enableComponent<components::WorldLinearVelocity>(
        _ecm, this->dataPtr->imuLink, true);

    // Ensure the rotor joints expose their velocity for the PID.
    for (auto &control : this->dataPtr->controls)
    {
      enableComponent<components::JointVelocity>(_ecm, control.joint, true);
    }
    return;
  }

  if (!_info.paused &&
      _info.simTime > this->dataPtr->lastControllerUpdateTime)
  {
    if (this->ReceiveMotorCommand() && this->dataPtr->betaflightOnline)
    {
      double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
          _info.simTime - this->dataPtr->lastControllerUpdateTime).count();
      this->ApplyMotorForces(dt, _ecm);
    }
  }
}

/////////////////////////////////////////////////
void gz::sim::systems::BetaFlightPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!_info.paused &&
      _info.simTime > this->dataPtr->lastControllerUpdateTime)
  {
    double t = std::chrono::duration_cast<std::chrono::duration<double>>(
        _info.simTime).count();
    this->SendState(t, _ecm);
    this->dataPtr->lastControllerUpdateTime = _info.simTime;
  }
}

/////////////////////////////////////////////////
bool gz::sim::systems::BetaFlightPlugin::ReceiveMotorCommand()
{
  servo_packet pkt;
  // Non-blocking: a short wait keeps the sim loop responsive.
  ssize_t recvSize = this->dataPtr->sock.recv(&pkt, sizeof(servo_packet), 1);

  // Drain the socket so we always act on the freshest command.
  while (true)
  {
    servo_packet last;
    ssize_t s = this->dataPtr->sock.recv(&last, sizeof(servo_packet), 0);
    if (s == -1)
    {
      break;
    }
    pkt = last;
    recvSize = s;
  }

  if (recvSize == -1)
  {
    return false;
  }

  if (recvSize < static_cast<ssize_t>(sizeof(servo_packet)))
  {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "received undersized motor packet (" << recvSize << " bytes).\n";
    return false;
  }

  if (!this->dataPtr->betaflightOnline)
  {
    gzlog << "[" << this->dataPtr->modelName << "] "
          << "Betaflight controller detected.\n";
    this->dataPtr->betaflightOnline = true;
  }

  for (auto &control : this->dataPtr->controls)
  {
    if (control.channel < 0 || control.channel >= 4)
    {
      continue;
    }
    const double raw = static_cast<double>(pkt.motor_speed[control.channel]);
    control.cmd = control.multiplier * (raw + control.offset);
  }
  return true;
}

/////////////////////////////////////////////////
void gz::sim::systems::BetaFlightPlugin::ApplyMotorForces(
    const double _dt,
    gz::sim::EntityComponentManager &_ecm)
{
  for (auto &control : this->dataPtr->controls)
  {
    auto *jfcComp =
        _ecm.Component<components::JointForceCmd>(control.joint);
    if (jfcComp == nullptr)
    {
      jfcComp = _ecm.CreateComponent(control.joint,
          components::JointForceCmd({0}));
    }

    const double velTarget = control.cmd / control.rotorVelocitySlowdownSim;
    auto *vComp = _ecm.Component<components::JointVelocity>(control.joint);
    if (vComp != nullptr && !vComp->Data().empty())
    {
      const double vel = vComp->Data()[0];
      const double error = vel - velTarget;
      const double force = control.pid.Update(
          error, std::chrono::duration<double>(_dt));
      jfcComp->Data()[0] = force;
    }
  }
}

/////////////////////////////////////////////////
void gz::sim::systems::BetaFlightPlugin::SendState(
    double _timestamp,
    const gz::sim::EntityComponentManager &_ecm)
{
  gz::msgs::IMU imuMsg;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
    if (!this->dataPtr->imuMsgValid)
    {
      return;
    }
    imuMsg = this->dataPtr->imuMsg;
  }

  // Gyro and accelerometer are already reported in the FRD body frame by the
  // imu_sensor (which is mounted rolled 180 deg in iris_with_standoffs).
  const gz::math::Vector3d angularVel(
      imuMsg.angular_velocity().x(),
      imuMsg.angular_velocity().y(),
      imuMsg.angular_velocity().z());
  const gz::math::Vector3d linearAccel(
      imuMsg.linear_acceleration().x(),
      imuMsg.linear_acceleration().y(),
      imuMsg.linear_acceleration().z());

  const auto *worldPose =
      _ecm.Component<components::WorldPose>(this->dataPtr->imuLink);
  const auto *worldLinearVel =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->imuLink);
  if (worldPose == nullptr || worldLinearVel == nullptr)
  {
    return;
  }

  // Pose / velocity into the ArduPilot/Betaflight NED world & FRD body frame.
  const gz::math::Pose3d bdyAToBdyG =
      this->dataPtr->modelXYZToAirplaneXForwardZDown.Inverse();
  const gz::math::Pose3d wldAToWldG = this->dataPtr->gazeboXYZToNED.Inverse();
  const gz::math::Pose3d wldGToBdyG = worldPose->Data();
  const gz::math::Pose3d wldAToBdyA =
      wldAToWldG * wldGToBdyG * bdyAToBdyG.Inverse();

  const gz::math::Vector3d velWldG = worldLinearVel->Data();
  const gz::math::Vector3d velWldA =
      wldAToWldG.Rot() * velWldG + wldAToWldG.Pos();

  const gz::math::Vector3d posNED = wldAToBdyA.Pos();
  const gz::math::Quaterniond q = wldAToBdyA.Rot();

  fdm_packet pkt;
  std::memset(&pkt, 0, sizeof(pkt));
  pkt.timestamp = _timestamp;

  pkt.imu_angular_velocity_rpy[0] = angularVel.X();
  pkt.imu_angular_velocity_rpy[1] = angularVel.Y();
  pkt.imu_angular_velocity_rpy[2] = angularVel.Z();

  pkt.imu_linear_acceleration_xyz[0] = linearAccel.X();
  pkt.imu_linear_acceleration_xyz[1] = linearAccel.Y();
  pkt.imu_linear_acceleration_xyz[2] = linearAccel.Z();

  pkt.imu_orientation_quat[0] = q.W();
  pkt.imu_orientation_quat[1] = q.X();
  pkt.imu_orientation_quat[2] = q.Y();
  pkt.imu_orientation_quat[3] = q.Z();

  pkt.velocity_xyz[0] = velWldA.X();
  pkt.velocity_xyz[1] = velWldA.Y();
  pkt.velocity_xyz[2] = velWldA.Z();

  pkt.position_xyz[0] = posNED.X();
  pkt.position_xyz[1] = posNED.Y();
  pkt.position_xyz[2] = posNED.Z();

  // Barometric pressure from altitude (NED z is down-positive).
  const double altitude = -posNED.Z();
  pkt.pressure =
      101325.0 * std::pow(1.0 - 2.25577e-5 * altitude, 5.25588);

  this->dataPtr->sock.sendto(&pkt, sizeof(pkt),
      this->dataPtr->fdm_address.c_str(), this->dataPtr->fdm_port_out);
}
