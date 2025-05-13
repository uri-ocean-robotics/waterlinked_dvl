#ifndef WATERLINKED_DVL_TCP_H
#define WATERLINKED_DVL_TCP_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "waterlinked_dvl/msg/transducer_report_stamped.hpp"
#include "waterlinked_dvl/msg/position_report_stamped.hpp"
#include "waterlinked_dvl/msg/transducer_stamped.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "thread"
#include "json/json.h"
#include "builtin_interfaces/msg/time.hpp"
#include "boost/thread/recursive_mutex.hpp"
#include "std_srvs/srv/set_bool.h"
#include "waterlinked_dvl/srv/get_config.hpp"

class WaterlinkedDvlTcp : public rclcpp::Node{
private:
    // Sockets
    boost::asio::io_service m_io_service;

    boost::asio::ip::tcp::socket m_socket;
    // Variables
    std::string m_frame_id;

    std::vector<double> m_velocity_covariance;

    std::vector<double> m_position_covariance;

    int m_time_type;

    std::string m_ip;

    int m_port;

    boost::recursive_mutex m_dynconf_lock;

    int m_speed_of_sound;

    int m_mounting_rotation_offset;

    bool m_acoustic_enabled;

    std::string m_range_mode;

    bool m_periodic_cycling_enabled;

    static constexpr double TO_RADIAN = 0.017453292519943295;
    // Thread
    boost::thread m_reading_thread;
    // Functions
    void f_read_loop();

    bool f_write(std::string &msg);

    void f_parse_and_publish(std::string incoming);

    void f_parse_json_v2(Json::Value root);

    void f_parse_json_v3(Json::Value root);

    double timeToSeconds(const builtin_interfaces::msg::Time& time_msg);
    // Publishers
    rclcpp::Publisher<waterlinked_dvl::msg::TransducerReportStamped>::SharedPtr m_transducer_report_publisher;

    rclcpp::Publisher<waterlinked_dvl::msg::TransducerReportStamped>::SharedPtr m_transducers_publisher;

    rclcpp::Publisher<waterlinked_dvl::msg::PositionReportStamped>::SharedPtr m_position_report_publisher;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_publisher;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr m_twist_publisher;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr m_raw_twist_publisher;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_altitude_publisher;

public:
    // Constructor
    WaterlinkedDvlTcp(rclcpp::NodeOptions options = rclcpp::NodeOptions());

};


#endif
