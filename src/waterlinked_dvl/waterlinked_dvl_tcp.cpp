#include "waterlinked_dvl_tcp.h"
#include "json/json.h"
#include "algorithm"
#include "string"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "waterlinked_dvl/PositionReportStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

WaterlinkedDvlTcp::WaterlinkedDvlTcp() :
        m_nh(),
        m_pnh("~"),
        m_socket(m_io_service)
{

    m_pnh.param<std::string>("ip", m_ip, "192.168.194.95"); // This value is set to fallback ip address
    m_pnh.param<int>("port", m_port, 16171);

    m_pnh.param<std::string>("frame_id", m_frame_id, "dvl_link");

    m_pnh.param<std::vector<double>>("velocity_covariance", m_velocity_covariance, std::vector<double>({}));

    m_pnh.param<std::vector<double>>("position_covariance", m_position_covariance, std::vector<double>({}));

    m_pnh.param<int>("speed_of_sound", m_speed_of_sound, 1500);

    m_pnh.param<int>("mounting_rotation_offset", m_mounting_rotation_offset, 0);

    m_pnh.param<bool>("acoustics_enabled", m_acoustic_enabled, false);

    m_twist_publisher = m_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("dvl/twist", 1000);

    m_pose_publisher = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("dvl/pose", 1000);

    m_transducer_report_publisher = m_nh.advertise<waterlinked_dvl::TransducerReportStamped>("dvl/transducer_report", 1000);

    m_position_report_publisher = m_nh.advertise<waterlinked_dvl::PositionReportStamped>("dvl/position_report", 1000);

    m_acoustics_enabled_service = m_nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
            "acoustics_enabled",
            boost::bind(
                    &WaterlinkedDvlTcp::f_callback_acoustics_enabled,
                    this,
                    boost::placeholders::_1,
                    boost::placeholders::_2
            )
    );

    m_get_last_response_service = m_nh.advertiseService<waterlinked_dvl::GetConfig::Request, waterlinked_dvl::GetConfig::Response>(
        "get_last_response",
        boost::bind(
            &WaterlinkedDvlTcp::f_callback_get_last_response,
            this,
            boost::placeholders::_1,
            boost::placeholders::_2
        )
    );

    m_get_running_config_service = m_nh.advertiseService<waterlinked_dvl::GetConfig::Request, waterlinked_dvl::GetConfig::Response>(
            "get_running_config",
            boost::bind(
                &WaterlinkedDvlTcp::f_callback_running_config,
                this,
                boost::placeholders::_1,
                boost::placeholders::_2
            )
    );

    m_dynconf_server = boost::make_shared<dynamic_reconfigure::Server<waterlinked_dvl::DVLConfig>>(m_dynconf_lock);

    m_dynconf_server->setCallback(boost::bind(&WaterlinkedDvlTcp::f_callback_dynconf, this, boost::placeholders::_1, boost::placeholders::_2));

    boost::asio::ip::tcp::endpoint  endpoint(
            boost::asio::ip::address::from_string(m_ip), m_port);

    m_socket.connect(endpoint);

    boost::asio::socket_base::receive_buffer_size option(81920);
    m_socket.set_option(option);

    m_reading_thread = boost::thread(
        boost::bind(&WaterlinkedDvlTcp::f_read_loop, this)
    );

    f_apply_config();
}


void WaterlinkedDvlTcp::f_read_loop() {
    boost::asio::streambuf incoming;
    while(ros::ok()) {
        boost::system::error_code error;
        std::size_t n_read = boost::asio::read_until(m_socket, incoming, '\n', error);


        if(error.value() != 0) {
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

bool WaterlinkedDvlTcp::f_write(std::string &msg) {

    ssize_t sent = boost::asio::write(m_socket, boost::asio::buffer(msg), boost::asio::transfer_all());

    return msg.size() == sent;
}

void WaterlinkedDvlTcp::f_parse_and_publish(std::string incoming) {


    Json::Value root;
    Json::Reader reader;

    try {
        reader.parse(incoming, root);

        auto format = root["format"].asString();
        if(format == "json_v2") {
            f_parse_json_v2(root);
        } else if (format == "json_v3") {
            f_parse_json_v3(root);
        } else {
            ROS_WARN_STREAM("Incoming message couldn't be parsed!: unknown format (" << format << ")");
        }

    } catch (Json::LogicError &e){
        ROS_WARN_STREAM("Incoming message couldn't be parsed!: " << e.what());
    }
}

void WaterlinkedDvlTcp::f_parse_json_v2(Json::Value root) {

    auto now = ros::Time::now();
    try{
        auto type = root["type"].asString();

        if(type == "velocity") {
            waterlinked_dvl::TransducerReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;

            msg.report.time = root["time"].asFloat();
            msg.report.altitude = root["altitude"].asFloat();
            msg.report.status = root["status"].asInt();
            msg.report.fom = root["fom"].asFloat();
            msg.report.velocity_valid = root["velocity_valid"].asBool();
            msg.report.vx = root["vx"].asFloat();
            msg.report.vy = root["vy"].asFloat();
            msg.report.vz = root["vz"].asFloat();

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

            m_transducer_report_publisher.publish(msg);

            if(msg.report.velocity_valid) {
                geometry_msgs::TwistWithCovarianceStamped twist_msg;
                twist_msg.header = msg.header;
                twist_msg.twist.twist.linear.x = msg.report.vx;
                twist_msg.twist.twist.linear.y = msg.report.vy;
                twist_msg.twist.twist.linear.z = msg.report.vz;

                if (m_velocity_covariance.size() == twist_msg.twist.covariance.size()) {
                    twist_msg.twist.covariance = as_array<twist_msg.twist.covariance.size()>(m_velocity_covariance);
                }
                m_twist_publisher.publish(twist_msg);
            }
        } else if (type == "position_local") {
            waterlinked_dvl::PositionReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;
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

            m_position_report_publisher.publish(msg);

            tf2::Quaternion quaternion;
            quaternion.setRPY(msg.report.roll * TO_RADIAN, msg.report.pitch * TO_RADIAN, msg.report.yaw * TO_RADIAN);
            quaternion.normalize();

            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose.pose.position.x = msg.report.x;
            pose_msg.pose.pose.position.y = msg.report.y;
            pose_msg.pose.pose.position.z = msg.report.z;

            pose_msg.pose.pose.orientation.w = quaternion.w();
            pose_msg.pose.pose.orientation.x = quaternion.x();
            pose_msg.pose.pose.orientation.y = quaternion.y();
            pose_msg.pose.pose.orientation.z = quaternion.z();

            pose_msg.pose.covariance[0] = pow(msg.report.std, 2);
            pose_msg.pose.covariance[6 * 1 + 1] = pow(msg.report.std, 2);
            pose_msg.pose.covariance[6 * 2 + 2] = pow(msg.report.std, 2);

            m_pose_publisher.publish(pose_msg);
        }
    } catch (Json::LogicError &e) {

    }

}

void WaterlinkedDvlTcp::f_parse_json_v3(Json::Value root)
{

    auto now = ros::Time::now();
    try
    {
        auto type = root["type"].asString();

        if(type == "velocity")
        {
            waterlinked_dvl::TransducerReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;

            msg.report.vx = root["vx"].asFloat();
            msg.report.vy = root["vy"].asFloat();
            msg.report.vz = root["vz"].asFloat();
            msg.report.fom = root["fom"].asFloat();
            msg.report.time = root["ts"].asFloat();

            int index = 0;
            for(Json::Value::ArrayIndex i = 0 ;  i != root["covariance"].size() ; i++) {
                for(Json::Value::ArrayIndex j = 0 ; j != root["covariance"][i].size() ; j ++) {
                    msg.report.covariance[index++] = root["covariance"][i][j].asFloat();
                }
            }

            msg.report.altitude = root["altitude"].asFloat();


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

            msg.report.format = root["format"].asString();

            msg.report.velocity_valid = root["velocity_valid"].asBool();

            msg.report.status = root["status"].asInt();

            msg.report.time_of_validity.fromNSec(root["time_of_validity"].asUInt64() * 1000U);

            msg.report.time_of_transmission.fromNSec(root["time_of_transmission"].asUInt64() * 1000U);

            m_transducer_report_publisher.publish(msg);

            if(msg.report.velocity_valid) {
                geometry_msgs::TwistWithCovarianceStamped twist_msg;
                twist_msg.header = msg.header;
                twist_msg.twist.twist.linear.x = msg.report.vx;
                twist_msg.twist.twist.linear.y = msg.report.vy;
                twist_msg.twist.twist.linear.z = msg.report.vz;

                if (m_velocity_covariance.size() == twist_msg.twist.covariance.size()) {
                    twist_msg.twist.covariance = as_array<twist_msg.twist.covariance.size()>(m_velocity_covariance);
                }
                m_twist_publisher.publish(twist_msg);
            }
        }
        else if (type == "position_local")
        {
            waterlinked_dvl::PositionReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;
            msg.report.x = root["x"].asFloat();
            msg.report.y = root["y"].asFloat();
            msg.report.z = root["z"].asFloat();

            msg.report.status = root["status"].asInt();
            msg.report.format = root["format"].asString();
            msg.report.roll = root["roll"].asFloat();
            msg.report.pitch = root["pitch"].asFloat();
            msg.report.yaw = root["yaw"].asFloat();
            msg.report.time = root["ts"].asFloat();
            msg.report.std = root["std"].asFloat();

            msg.report.format = root["format"].asString();

            m_position_report_publisher.publish(msg);

            tf2::Quaternion quaternion;
            quaternion.setRPY(msg.report.roll * TO_RADIAN, msg.report.pitch * TO_RADIAN, msg.report.yaw * TO_RADIAN);
            quaternion.normalize();

            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose.pose.position.x = msg.report.x;
            pose_msg.pose.pose.position.y = msg.report.y;
            pose_msg.pose.pose.position.z = msg.report.z;

            pose_msg.pose.pose.orientation.w = quaternion.w();
            pose_msg.pose.pose.orientation.x = quaternion.x();
            pose_msg.pose.pose.orientation.y = quaternion.y();
            pose_msg.pose.pose.orientation.z = quaternion.z();

            pose_msg.pose.covariance[0] = pow(msg.report.std, 2);
            pose_msg.pose.covariance[6 * 1 + 1] = pow(msg.report.std, 2);
            pose_msg.pose.covariance[6 * 2 + 2] = pow(msg.report.std, 2);

            m_pose_publisher.publish(pose_msg);
        }
        else if (type == "response")
        {

            auto response_to = root["response_to"].asString();
            if(response_to == "set_config") {
                m_last_response.response_to = response_to;
                m_last_response.success = root["success"].asBool();
                m_last_response.error_message = root["error_message"].asString();
                m_last_response.type = type;
                m_last_response.format = root["format"].asString();
            } else if (response_to == "get_config") {
                m_running_config.response_to = response_to;
                m_running_config.success = root["success"].asBool();
                m_running_config.error_message = root["error_message"].asString();
                m_running_config.type = type;
                m_running_config.format = root["format"].asString();
                m_running_config.speed_of_sound = root["result"]["speed_of_sound"].asInt();
                m_running_config.acoustic_enabled = root["result"]["acoustic_enabled"].asBool();
                m_running_config.mounting_rotation = root["result"]["mounting_rotation_offset"].asInt();
            } else {

            }
        }
    } catch (Json::LogicError &e) {

    }

}

void WaterlinkedDvlTcp::f_amend_dynconf(int speed_of_sound, int mounting_offset, bool acoustic_enabled) {
    boost::recursive_mutex::scoped_lock lock(m_dynconf_lock);

    waterlinked_dvl::DVLConfig conf;

    conf.speed_of_sound = m_speed_of_sound;
    conf.acoustic_enabled = m_acoustic_enabled;
    conf.mounting_rotation_offset = m_mounting_rotation_offset;

    m_dynconf_server->updateConfig(conf);
}

void WaterlinkedDvlTcp::f_callback_dynconf(waterlinked_dvl::DVLConfig &config, uint32_t level) {
    if(m_acoustic_enabled != config.acoustic_enabled) {
        f_amend_acoustic_enabled(config.acoustic_enabled);
        m_acoustic_enabled = config.acoustic_enabled;
    }

    if(m_mounting_rotation_offset != config.mounting_rotation_offset) {
        f_amend_mounting_rotation(config.mounting_rotation_offset);
        m_mounting_rotation_offset = config.mounting_rotation_offset;
    }

    if(m_speed_of_sound != config.speed_of_sound) {
        f_amend_sound_speed(config.speed_of_sound);
        m_speed_of_sound = config.speed_of_sound;
    }

}

bool WaterlinkedDvlTcp::f_amend_acoustic_enabled(bool enabled) {

    Json::Value config_msg;

    config_msg["command"] = "set_config";

    Json::Value parameters;
    parameters["acoustic_enabled"] = enabled;
    config_msg["parameters"] = parameters;


    Json::FastWriter fast_writer;

    auto msg = fast_writer.write(config_msg);

    return f_write(msg);
}

bool WaterlinkedDvlTcp::f_amend_sound_speed(int speed_of_sound) {

    Json::Value config_msg;

    config_msg["command"] = "set_config";

    Json::Value parameters;
    parameters["speed_of_sound"] = speed_of_sound;
    config_msg["parameters"] = parameters;

    Json::FastWriter fast_writer;

    auto msg = fast_writer.write(config_msg);

    return f_write(msg);
}

bool WaterlinkedDvlTcp::f_amend_mounting_rotation(int rotation) {

    Json::Value config_msg;

    config_msg["command"] = "set_config";

    Json::Value parameters;
    parameters["mounting_rotation_offset"] = rotation;
    config_msg["parameters"] = parameters;

    Json::FastWriter fast_writer;

    auto msg = fast_writer.write(config_msg);

    return f_write(msg);
}

bool WaterlinkedDvlTcp::f_callback_acoustics_enabled(std_srvs::SetBool::Request &req,
                                                     std_srvs::SetBool::Response &res) {

    m_last_response.response_to.clear();
    res.message = std::string() + "Acoustics " + (req.data ? "enabled" : "disabled");
    bool result = f_amend_acoustic_enabled(req.data);
    if(!result) {
        res.success = result;
        return false;
    }
    ros::Rate r(10);
    while(m_last_response.response_to != "set_config") {
        r.sleep();
    }
    res.success = m_last_response.success;
    return res.success;
}

bool WaterlinkedDvlTcp::f_callback_get_last_response(waterlinked_dvl::GetConfig::Request &req,
                                                     waterlinked_dvl::GetConfig::Response &res) {

    res.config = m_last_response;

    return true;
}

bool WaterlinkedDvlTcp::f_callback_running_config(waterlinked_dvl::GetConfig::Request &req,
                                                  waterlinked_dvl::GetConfig::Response &res) {

    f_acquire_running_config();
    ros::Rate r(10);
    while (m_running_config.response_to != "get_config") {
        r.sleep();
    }
    res.config = m_running_config;

    m_running_config.response_to.clear();

    return true;
}

bool WaterlinkedDvlTcp::f_acquire_running_config() {
    Json::Value config_msg;

    config_msg["command"] = "get_config";

    Json::FastWriter fast_writer;

    auto msg = fast_writer.write(config_msg);

    return f_write(msg);
}

void WaterlinkedDvlTcp::f_apply_config() {

    ros::Rate r(1);

    while (true) {
        f_acquire_running_config();
        r.sleep();

        if(m_running_config.speed_of_sound != m_speed_of_sound) {
            f_amend_sound_speed(m_speed_of_sound);
        }

        if(m_running_config.mounting_rotation != m_mounting_rotation_offset) {
            f_amend_mounting_rotation(m_mounting_rotation_offset);
        }

        if(m_running_config.acoustic_enabled != m_acoustic_enabled) {
            f_amend_acoustic_enabled(m_acoustic_enabled);
        }

        if(
                m_running_config.speed_of_sound == m_speed_of_sound &&
                m_running_config.acoustic_enabled == m_acoustic_enabled &&
                m_running_config.mounting_rotation == m_mounting_rotation_offset
                ) {
            break;
        }
    }

    f_amend_dynconf(m_speed_of_sound, m_mounting_rotation_offset, m_acoustic_enabled);

}