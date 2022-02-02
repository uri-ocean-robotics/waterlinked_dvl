#ifndef WATERLINKED_DVL_TCP_H
#define WATERLINKED_DVL_TCP_H

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "thread"
#include "json/json.h"
#include "boost/thread/recursive_mutex.hpp"
#include "waterlinked_dvl/DVLConfig.h"
#include "dynamic_reconfigure/server.h"
#include "std_srvs/SetBool.h"
#include "waterlinked_dvl/Config.h"
#include "waterlinked_dvl/GetConfig.h"

class WaterlinkedDvlTcp {
private:

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    boost::asio::io_service m_io_service;

    boost::asio::ip::tcp::socket m_socket;

    std::string m_frame_id;

    std::vector<double> m_velocity_covariance;

    std::vector<double> m_position_covariance;

    boost::shared_ptr<dynamic_reconfigure::Server<waterlinked_dvl::DVLConfig>> m_dynconf_server;

    std::string m_ip;

    int m_port;

    void f_read_loop();

    bool f_write(std::string &msg);

    void f_parse_and_publish(std::string incoming);

    void f_parse_json_v2(Json::Value root);

    void f_parse_json_v3(Json::Value root);

    boost::thread m_reading_thread;

    ros::Publisher m_transducer_report_publisher;

    ros::Publisher m_pose_publisher;

    ros::Publisher m_position_report_publisher;

    ros::Publisher m_twist_publisher;

    boost::recursive_mutex m_dynconf_lock;

    ros::ServiceServer m_acoustics_enabled_service;

    ros::ServiceServer m_get_last_response_service;

    ros::ServiceServer m_get_running_config_service;

    int m_speed_of_sound;

    int m_mounting_rotation_offset;

    bool m_acoustic_enabled;

    void f_amend_dynconf(int speed_of_sound, int mounting_offset, bool acoustic_enabled);

    void f_callback_dynconf(waterlinked_dvl::DVLConfig &config, uint32_t level);

    waterlinked_dvl::Config m_last_response;

    waterlinked_dvl::Config m_running_config;

    bool f_callback_acoustics_enabled(std_srvs::SetBool::Request& req,
                                      std_srvs::SetBool::Response& res);

    bool f_callback_get_last_response(waterlinked_dvl::GetConfig::Request& req,
                                      waterlinked_dvl::GetConfig::Response& res);

    bool f_amend_sound_speed(int speed_of_sound);

    bool f_amend_acoustic_enabled(bool enabled);

    bool f_amend_mounting_rotation(int rotation);

    bool f_callback_running_config(waterlinked_dvl::GetConfig::Request& req,
                                   waterlinked_dvl::GetConfig::Response& res);

    bool f_acquire_running_config();

    void f_apply_config();

    static constexpr double TO_RADIAN = 0.017453292519943295;

public:

    WaterlinkedDvlTcp();

};


#endif
