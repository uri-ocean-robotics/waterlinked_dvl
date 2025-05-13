#ifndef WATERLINKED_DVL_SERIAL_H
#define WATERLINKED_DVL_SERIAL_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "waterlinked_dvl/msg/transducer_report_stamped.hpp"
#include "waterlinked_dvl/msg/position_report_stamped.hpp"
#include "waterlinked_dvl/msg/transducer_stamped.hpp"
#include "memory"
#include "boost/asio.hpp"

class WaterlinkedDvlSerial : public rclcpp::Node{

private:
    std::string m_frame_id_;
    
    std::string port_;
    
    boost::asio::io_service m_io;

    boost::asio::serial_port m_ser_port;

    std::vector<double> m_velocity_covariance;

    std::vector<double> m_position_covariance;

    rclcpp::Publisher<waterlinked_dvl::msg::TransducerReportStamped>::SharedPtr m_transducer_report_publisher;

    rclcpp::Publisher<waterlinked_dvl::msg::TransducerReportStamped>::SharedPtr m_transducers_publisher;

    rclcpp::Publisher<waterlinked_dvl::msg::PositionReportStamped>::SharedPtr m_position_report_publisher;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_publisher;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr m_twist_publisher;

    bool f_checksum(const std::string& incoming);

    std::string f_serial_read();

    void f_serial_parse(std::string incoming);

    void f_clean_up();

public:
    WaterlinkedDvlSerial(rclcpp::NodeOptions options = rclcpp::NodeOptions());



};


#endif