// Compile with:
// g++ -shared -fPIC wave_motion_plugin.cc -o libWaveMotionPlugin.so `pkg-config --cflags --libs gz-sim8` && mv ./libWaveMotionPlugin.so ../../build/

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>

#include <chrono>
#include <cmath>
#include <iostream>

namespace wave_sim
{
    class WaveMotionPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> & _sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override
    {
        this->model = gz::sim::Model(_entity);

        // Read SDF parameters
        if (_sdf->HasElement("amplitude"))
        {
            this->amplitude = _sdf->Get<double>("amplitude");
        }
        if (_sdf->HasElement("frequency"))
        {
            this->frequency = _sdf->Get<double>("frequency");
        }

        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
        if (poseComp)
        {
            this->startPose = poseComp->Data();
        }
        else
        {
            this->startPose = gz::math::Pose3d();
        }
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused)
        {
            return;
        }

        auto t = std::chrono::duration<double>(_info.simTime).count();

        // Wave motion parameters
        double base_z = startPose.Pos().Z();
        double z = base_z + amplitude * std::sin(2 * M_PI * frequency * t);
        double roll = 0.05 * std::sin(2 * M_PI * 0.05 * t);
        double pitch = 0.05 * std::sin(2 * M_PI * 0.1 * t);

        gz::math::Pose3d newPose(startPose.Pos().X(), startPose.Pos().Y(), z, roll, pitch, 0);
        model.SetWorldPoseCmd(_ecm, newPose);
    }

    private:
        gz::sim::Model model;
        gz::math::Pose3d startPose;
        double amplitude{1.0};
        double frequency{0.5};
    };
}

GZ_ADD_PLUGIN(wave_sim::WaveMotionPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(wave_sim::WaveMotionPlugin, "wave_motion_plugin")
