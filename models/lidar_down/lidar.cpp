// Compile with:
// g++ -shared -fPIC lidar.cpp -o libLidarPlugin.so `pkg-config --cflags --libs gz-sim8` && mv ./libLidarPlugin.so ../../build/

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sensors/Lidar.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/lidar_sensor.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <mavlink/v2.0/common/mavlink.h>
#include <boost/asio.hpp>

#include <atomic>


namespace gz::sim
{

class DistanceSensorMavlinkPlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
    void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &sdf,
                    EntityComponentManager &_ecm, EventManager &_eventMgr) override
    {
        std::vector<std::string> models;
        Entity current = _entity;
        while (current != kNullEntity)
        {
            const components::Name *name_comp = _ecm.Component<components::Name>(current);
            if (name_comp)
            {
                models.push_back(name_comp->Data());
            }

            current = _ecm.ParentEntity(current);
        }

        // We find the first parent last therefore we need to reverse the order.
        std::string models_path;
        for (auto riter = models.rbegin(); riter != models.rend(); ++riter)
        {
            models_path += "model/" + *riter + "/";
        }

        Entity world_entity = kNullEntity;
        _ecm.Each<components::World>(
            [&](const Entity &world__entity, const components::World *) -> bool
            {
            world_entity = world__entity;
            return false; // stop after finding the first world
            }
        );

        std::string world_name;
        const std::optional<std::string> world_data = _ecm.ComponentData<components::Name>(world_entity);
        if (world_data.has_value())
        {
            world_name = world_data.value() + "/";
        }

        m_sensor_topic = "/world/" + world_name + models_path + "link/lidar_link/sensor/lidar/scan";

        if (sdf->HasElement("ip"))
        {
            m_ip = sdf->Get<std::string>("ip");
        }

        if (sdf->HasElement("port"))
        {
            m_port = sdf->Get<int>("port");
        }

        if (sdf->HasElement("rate"))
        {
            m_message_rate = sdf->Get<size_t>("rate");
        }

        if (sdf->HasElement("sensor_topic"))
        {
            m_sensor_topic = sdf->Get<std::string>("sensor_topic");
        }

        m_socket.open(boost::asio::ip::udp::v4());
        m_remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_ip), m_port);

        m_node.Subscribe(m_sensor_topic, &DistanceSensorMavlinkPlugin::OnLidarMessage, this);
        gzmsg << "Initialized LIDAR on topic: " << m_sensor_topic << " sending mavlink to: " << m_ip << ":" << m_port << std::endl;

        // Blackout control service: absolute mode-set, req = Int32 mode, rep = Boolean accepted.
        m_blackout_service = "/" + models_path + "blackout";
        if (sdf->HasElement("blackout_service"))
        {
            m_blackout_service = sdf->Get<std::string>("blackout_service");
        }

        if (!m_node.Advertise(m_blackout_service, &DistanceSensorMavlinkPlugin::OnBlackoutService, this))
        {
            gzerr << "Failed to advertise LIDAR blackout service on: " << m_blackout_service << std::endl;
        }
        else
        {
            gzmsg << "LIDAR blackout service on: " << m_blackout_service
                  << " (0=NORMAL, 1=SILENCE, 2=OUT_OF_RANGE)" << std::endl;
        }
    }

    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override {}

private:
    void OnLidarMessage(const msgs::LaserScan &_msg)
    {
        if (_msg.ranges_size() == 0)
        {
            return;
        }

        static std::chrono::steady_clock::time_point old_time = std::chrono::steady_clock::now();
        const std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
        const std::chrono::duration<float, std::milli> dt = current_time - old_time;
        if (dt.count() > 1000.0f / m_message_rate)
        {
            old_time = current_time;

            const int mode = m_mode.load(std::memory_order_relaxed);
            if (mode == SILENCE)
            {
                return; // blackout: drop the packet so ArduPilot sees the rangefinder time out
            }

            mavlink_distance_sensor_t dist_sensor_msg;
            dist_sensor_msg.time_boot_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - boot_time).count();
            dist_sensor_msg.min_distance = static_cast<uint16_t>(_msg.range_min() * 100);
            dist_sensor_msg.max_distance = static_cast<uint16_t>(_msg.range_max() * 100);
            dist_sensor_msg.current_distance = static_cast<uint16_t>(std::clamp(_msg.ranges(0), _msg.range_min(), _msg.range_max()) * 100);
            if (mode == OUT_OF_RANGE)
            {
                // Healthy sensor reporting no return: ArduPilot flags this OutOfRangeHigh.
                dist_sensor_msg.current_distance = static_cast<uint16_t>(dist_sensor_msg.max_distance + 1);
            }

            dist_sensor_msg.type = MAV_DISTANCE_SENSOR_LASER;
            dist_sensor_msg.id = 0;
            dist_sensor_msg.orientation = MAV_SENSOR_ROTATION_PITCH_270;
            dist_sensor_msg.covariance = 0.0;

            mavlink_message_t msg;
            mavlink_msg_distance_sensor_encode(1, 1, &msg, &dist_sensor_msg);
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            int len = mavlink_msg_to_send_buffer(buf, &msg);
            m_socket.send_to(boost::asio::buffer(buf, len), m_remote_endpoint);
        }
    }

    // Absolute mode-set service callback. Runs on a transport thread.
    bool OnBlackoutService(const msgs::Int32 &_req, msgs::Boolean &_rep)
    {
        const int requested = _req.data();
        if (requested < NORMAL || requested > OUT_OF_RANGE)
        {
            gzwarn << "LIDAR blackout: rejected invalid mode " << requested << std::endl;
            _rep.set_data(false);
            return true;
        }

        setMode(requested);
        _rep.set_data(true);
        return true;
    }

    void setMode(int mode)
    {
        const int previous = m_mode.exchange(mode, std::memory_order_relaxed);
        if (previous != mode)
        {
            gzmsg << "LIDAR blackout mode: " << modeName(previous) << " -> " << modeName(mode) << std::endl;
        }
    }

    static const char *modeName(int mode)
    {
        switch (mode)
        {
            case NORMAL:       return "NORMAL";
            case SILENCE:      return "SILENCE";
            case OUT_OF_RANGE: return "OUT_OF_RANGE";
            default:           return "UNKNOWN";
        }
    }

    // Use in bash to trigger lidar failure modes:
    // gz service -s {SERVICE_NAME} --reqtype gz.msgs.Int32 --reptype gz.msgs.Boolean --req 'data: 1'
    // Find the service name with `gz service -l`
    enum BlackoutMode { NORMAL = 0, SILENCE = 1, OUT_OF_RANGE = 2 };

    std::atomic<int> m_mode{NORMAL};
    std::string m_blackout_service;

    transport::Node m_node;
    std::string m_sensor_topic;
    std::string m_ip = "127.0.0.1";
    int m_port = 14550;
    size_t m_message_rate = 10;
    const std::chrono::steady_clock::time_point boot_time = std::chrono::steady_clock::now();

    boost::asio::io_service m_io_service;
    boost::asio::ip::udp::socket m_socket{m_io_service};
    boost::asio::ip::udp::endpoint m_remote_endpoint;
};

GZ_ADD_PLUGIN(DistanceSensorMavlinkPlugin,
    System,
    ISystemConfigure,
    ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(DistanceSensorMavlinkPlugin, "lidar_plugin")
}
