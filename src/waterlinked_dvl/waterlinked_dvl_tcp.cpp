#include "waterlinked_dvl_tcp.h"
#include "json/json.h"
#include "algorithm"
#include "string"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "waterlinked_dvl/PositionReportStamped.h"

WaterlinkedDvlTcp::WaterlinkedDvlTcp() :
        m_nh(),
        m_pnh("~"),
        m_socket(m_io_context)
{

    m_pnh.param<std::string>("ip", m_ip, "192.168.194.95"); // This value is set to fallback ip address
    m_pnh.param<int>("port", m_port, 16171);

    m_transducer_publisher = m_nh.advertise<waterlinked_dvl::TransducerReportStamped>("transducer_report",1000);

    m_position_publisher = m_nh.advertise<waterlinked_dvl::PositionReportStamped>("position_report",1000);

    boost::asio::ip::tcp::endpoint  endpoint(
            boost::asio::ip::address::from_string(m_ip), m_port);

    m_socket.connect(endpoint);

    boost::asio::socket_base::receive_buffer_size option(81920);
    m_socket.set_option(option);

    m_reading_thread = boost::thread(
        boost::bind(&WaterlinkedDvlTcp::f_readloop, this)
    );
}


void WaterlinkedDvlTcp::f_readloop() {
    boost::asio::streambuf incoming;
    while(ros::ok()) {
        boost::system::error_code error;
        std::size_t n_read = boost::asio::read_until(m_socket, incoming, '\n', error);


        if(error.failed()) {
            continue;
        }

        boost::asio::streambuf::const_buffers_type bufs = incoming.data();
        std::string str(boost::asio::buffers_begin(bufs),
                        boost::asio::buffers_begin(bufs) + n_read);

        incoming.consume(n_read);
        boost::thread t{
            boost::bind(&WaterlinkedDvlTcp::f_parse_and_publish, this, str)
        };

    }
}

void WaterlinkedDvlTcp::f_parse_and_publish(std::string incoming) {

    auto now = ros::Time::now();

    Json::Value root;
    Json::Reader reader;

    try {
        reader.parse(incoming, root);

        auto type = root["type"];
        if(type == "velocity") {
            waterlinked_dvl::TransducerReportStamped msg;
            msg.header.stamp = now;

            msg.report.time = root["time"].asFloat();
            msg.report.altitude = root["altitude"].asFloat();
            msg.report.status = root["status"].asInt();
            msg.report.fom = root["fom"].asFloat();
            msg.report.velocity_valid = root["velocity_valid"].asBool();
            msg.report.vx = root["vx"].asFloat();
            msg.report.vy = root["vy"].asFloat();
            msg.report.vz = root["vz"].asFloat();
            msg.report.format = root["format"].asString();

            auto transducers_json = root["transducers"];
            for (Json::Value::ArrayIndex i = 0; i != transducers_json.size(); i++) {
                waterlinked_dvl::Transducer transducer;

                transducer.distance = transducers_json[i]["distance"].asFloat();
                transducer.id = transducers_json[i]["id"].asInt();
                transducer.beam_valid = transducers_json[i]["beam_valid"].asBool();

                transducer.nsd = transducers_json[i]["nsd"].asFloat();
                transducer.rssi = transducers_json[i]["rssi"].asFloat();
                transducer.velocity = transducers_json[i]["velocity"].asFloat();

                msg.report.transducers.push_back(transducer);
            }

            m_transducer_publisher.publish(msg);
        } else if (type == "position_local") {
            waterlinked_dvl::PositionReportStamped msg;
            msg.header.stamp = now;

            msg.report.x = root["x"].asFloat();
            msg.report.y = root["y"].asFloat();
            msg.report.z = root["z"].asFloat();

            msg.report.status = root["status"].asInt();
            msg.report.format = root["format"].asString();
            msg.report.roll = root["roll"].asFloat();
            msg.report.pitch = root["pitch"].asFloat();
            msg.report.yaw = root["yaw"].asFloat();
            msg.report.time = root["time"].asFloat();
            msg.report.std = root["std"].asFloat();

            m_position_publisher.publish(msg);
        } else {
            ROS_WARN("Incoming message doesn't have a valid type");
        }

    } catch (Json::LogicError &e){
        ROS_WARN_STREAM("Incoming message couldn't be parsed!: " << e.what());
    }
}
