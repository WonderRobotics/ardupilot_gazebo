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
#ifndef BETAFLIGHTPLUGIN_HH_
#define BETAFLIGHTPLUGIN_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration of the private data class.
  class BetaFlightPluginPrivate;

  /// \brief Interface Betaflight from ardupilot stack.
  ///
  /// This plugin bridges a Gazebo model to a Betaflight SITL flight
  /// controller using the Betaflight SITL UDP protocol (see
  /// betaflight/src/platform/SIMULATOR/target/SITL/target.h):
  ///
  ///   * Gazebo -> Betaflight: `fdm_packet` on UDP port 9003 (default).
  ///     Carries the simulated IMU, attitude, velocity and position in the
  ///     ArduPilot/Betaflight NED world / FRD body convention.
  ///   * Betaflight -> Gazebo: `servo_packet` on UDP port 9002 (default).
  ///     Carries 4 normalised (0..1) motor commands.
  ///
  /// The frame conventions and per-rotor control setup are identical to
  /// ArduPilotPlugin, so the same iris_with_standoffs rotor mapping applies:
  /// motor_speed[i] drives `<control channel="i">`.
  class BetaFlightPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: BetaFlightPlugin();

    /// \brief Destructor.
    public: ~BetaFlightPlugin();

    // Documentation inherited.
    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) final;

    // Documentation inherited.
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) final;

    // Documentation inherited.
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Load the control channels from sdf.
    private: void LoadControlChannels(
        sdf::ElementPtr _sdf,
        gz::sim::EntityComponentManager &_ecm);

    /// \brief Bind the UDP socket used to talk to Betaflight.
    private: bool InitSockets(sdf::ElementPtr _sdf) const;

    /// \brief Receive a motor command packet from Betaflight (non-blocking).
    private: bool ReceiveMotorCommand();

    /// \brief Apply the latest motor commands to the rotor joints.
    private: void ApplyMotorForces(
        const double _dt,
        gz::sim::EntityComponentManager &_ecm);

    /// \brief Build and send the flight-dynamics state to Betaflight.
    private: void SendState(
        double _timestamp,
        const gz::sim::EntityComponentManager &_ecm);

    /// \brief Private data pointer.
    private: std::unique_ptr<BetaFlightPluginPrivate> dataPtr;
  };
}
}
}
}

#endif  // BETAFLIGHTPLUGIN_HH_
