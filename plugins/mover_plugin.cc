// Compile with:
// g++ -shared -fPIC mover_plugin.cc -o libMoverPlugin.so `pkg-config --cflags --libs gz-sim8` && mv ./libMoverPlugin.so ../build/

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

namespace mover
{
    class MoverPlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
    {
    public:
        void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) override
        {
            m_ecm = &_ecm;

            // Read SDF parameters
            if (_sdf->HasElement("model"))
            {
                m_model_name = _sdf->Get<std::string>("model");
            }
            else
            {
                gzerr << "Model parameter is missing for Mover plugin." << std::endl;
                return;
            }

            if (_sdf->HasElement("topic"))
            {
                m_topic = _sdf->Get<std::string>("topic");
            }
            else
            {
                gzerr << "Topic parameter is missing for Mover plugin." << std::endl;
                return;
            }

            m_first_update = true;

            m_node.Subscribe(m_topic, &MoverPlugin::OnMsg, this);
            gzmsg << "Configured MoverPlugin with model '" << m_model_name << "' and topic '" << m_topic << "'" << std::endl;
        }

        void OnMsg(const gz::msgs::Pose &_msg)
        {
            m_position = _msg.position();
            m_orientation = _msg.orientation();
        }

        void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
        {
            const gz::sim::Entity model_id = _ecm.EntityByComponents(gz::sim::components::Name(m_model_name));

            if (m_first_update)
            {
                m_first_update = false;
                const auto pose = _ecm.Component<gz::sim::components::Pose>(model_id);
                if (pose)
                {
                    m_start_pose = pose->Data();
                }
                else
                {
                    m_start_pose = gz::math::Pose3d();
                }
            }

            if (_info.paused)
            {
                return;
            }

            if (model_id != gz::sim::kNullEntity)
            {
                gz::sim::Model model{model_id};
                auto eulers = gz::math::Quaternion{m_orientation.w(), m_orientation.x(), m_orientation.y(), m_orientation.z()}.Euler();
                gz::math::Pose3d newPose{
                    m_start_pose.Pos().X() + m_position.x(),
                    m_start_pose.Pos().Y() + m_position.y(),
                    m_start_pose.Pos().Z() + m_position.z(),
                    eulers.X(),
                    eulers.Y(),
                    eulers.Z()
                };

                model.SetWorldPoseCmd(_ecm, newPose);
            }
        }

    private:
        std::string m_topic;
        std::string m_model_name;
        bool m_first_update;
        gz::math::Pose3d m_start_pose;
        gz::transport::Node m_node;
        gz::sim::EntityComponentManager *m_ecm{nullptr};
        gz::msgs::Vector3d m_position;
        gz::msgs::Quaternion m_orientation;
    };
}

GZ_ADD_PLUGIN(mover::MoverPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(mover::MoverPlugin, "mover_plugin")
