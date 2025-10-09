// Compile with:
// g++ -shared -fPIC path_mover_plugin.cc -o libPathMoverPlugin.so `pkg-config --cflags --libs gz-sim8` && mv ./libPathMoverPlugin.so ../build/

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

#include <regex>

using point = std::pair<float, float>;

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
            // Read SDF parameters
            if (_sdf->HasElement("speed"))
            {
                m_speed = _sdf->Get<float>("speed");
            }

            if (_sdf->HasElement("angularSpeed"))
            {
                m_angular_speed = _sdf->Get<float>("angularSpeed");
            }

            if (_sdf->HasElement("angleOffset"))
            {
                m_angle_offset = _sdf->Get<float>("angleOffset") * M_PI / 180.0f;
            }

            if (_sdf->HasElement("model"))
            {
                m_model_name = _sdf->Get<std::string>("model");
            }
            else
            {
                gzerr << "Model parameter is missing for Path Mover plugin." << std::endl;
                return;
            }

            if (_sdf->HasElement("points"))
            {

                m_points = parse_points(_sdf->Get<std::string>("points"));
            }
            else
            {
                gzerr << "Points parameter is missing for Path Mover plugin." << std::endl;
                return;
            }

            m_first_update = true;

            std::string initialization_string = "Configured Path Mover plugin with model '" + m_model_name + "' and speed of " + std::to_string(m_speed) + " [m/s] and points: [";
            for (size_t i = 0; i < m_points.size(); i++)
            {
                if (i == m_points.size() - 1)
                {
                    initialization_string += std::to_string(m_points[i].first) + ", " + std::to_string(m_points[i].second) + ")]\n";
                }
                else
                {
                    initialization_string += "(" + std::to_string(m_points[i].first) + ", " + std::to_string(m_points[i].second) + "), ";
                }
            }

            gzmsg << initialization_string;
        }

        void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
        {
            const gz::sim::Entity model_id = _ecm.EntityByComponents(gz::sim::components::Name(m_model_name));
            const float dt = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime - m_last_time).count();
            m_last_time = _info.simTime;

            if (m_first_update)
            {
                m_first_update = false;
                const auto pose = _ecm.Component<gz::sim::components::Pose>(model_id);
                if (pose)
                {
                    const auto start_pose = pose->Data();
                    m_position.set_x(start_pose.Pos().X());
                    m_position.set_y(start_pose.Pos().Y());
                    m_position.set_z( start_pose.Pos().Z());
                    m_orientation.set_x(start_pose.Roll());
                    m_orientation.set_y(start_pose.Pitch());
                    m_orientation.set_z(start_pose.Yaw());
                }
            }

            if (_info.paused)
            {
                return;
            }

            const float travel_distance = m_speed * dt / 1e6f;
            const auto next_point = m_points[get_next_index()];
            const float dx = next_point.first - m_position.x();
            const float dy = next_point.second - m_position.y();
            const float points_distance = std::sqrt(dx * dx + dy * dy);
            m_position.set_x(m_position.x() + travel_distance * dx / points_distance);
            m_position.set_y(m_position.y() + travel_distance * dy / points_distance);
            if (calc_distance({m_position.x(), m_position.y()}, next_point) < 1)
            {
                // Arrived!
                m_index = get_next_index();
            }

            const float max_turn_step = m_angular_speed * dt / 1e6f;
            const float target_angle = std::atan2(dy, dx) + m_angle_offset;
            float angle_diff = target_angle - m_orientation.z();
            while (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }

            while (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }

            const float turn_step = std::clamp(angle_diff, -max_turn_step, max_turn_step);
            m_orientation.set_z(m_orientation.z() + turn_step);

            if (model_id != gz::sim::kNullEntity)
            {
                gz::sim::Model model{model_id};
                gz::math::Pose3d newPose{
                    m_position.x(),
                    m_position.y(),
                    m_position.z(),
                    m_orientation.x(),
                    m_orientation.y(),
                    m_orientation.z()
                };

                model.SetWorldPoseCmd(_ecm, newPose);
            }
        }

    private:
        static std::vector<point> parse_points(const std::string& input)
        {
            std::vector<point> points;
            std::regex re(R"(\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\))");
            std::smatch match;

            auto it = input.cbegin();
            while (std::regex_search(it, input.cend(), match, re))
            {
                const float x = std::stod(match[1].str());
                const float y = std::stod(match[2].str());
                points.push_back({x, y});
                it = match.suffix().first;
            }

            return points;
        }

        size_t get_next_index() const
        {
            return (m_index + 1) % m_points.size();
        }

        static double calc_distance(const point& p1, const point& p2) {
            const float dx = p1.first - p2.first;
            const float dy = p1.second - p2.second;
            return std::sqrt(dx * dx + dy * dy);
        }

        std::vector<point> m_points;
        std::string m_model_name;
        bool m_first_update;
        gz::msgs::Vector3d m_position;
        gz::msgs::Vector3d m_orientation;
        float m_speed = 10; // [m/s]
        float m_angular_speed = 3; // [rad/s]
        float m_angle_offset = 0; // degrees
        std::chrono::steady_clock::duration m_last_time{0};
        size_t m_index = 0;
    };
}

GZ_ADD_PLUGIN(mover::MoverPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(mover::MoverPlugin, "path_mover_plugin")
