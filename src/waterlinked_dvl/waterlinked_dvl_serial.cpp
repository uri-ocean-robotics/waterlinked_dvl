#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include "waterlinked_dvl_serial.h"
#include <rclcpp/rclcpp.hpp>
#include "boost/crc.hpp"
#include "thread"
#include "utils.hpp"


WaterlinkedDvlSerial::WaterlinkedDvlSerial(rclcpp::NodeOptions options): 
        Node("waterlinked_dvl_node", options),
        m_io(),
        m_ser_port(m_io)  
    {
    this->declare_parameter<std::string>("port");
    port_ = this->get_parameter("port").as_string();

    this->declare_parameter<std::string>("frame_id");
    m_frame_id_ = this->get_parameter("frame_id").as_string();

    m_ser_port.open(port_);
    
    m_ser_port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    
    m_position_report_publisher = create_publisher<waterlinked_dvl::msg::PositionReportStamped>("dvl/position_report", 1000);
    m_transducers_publisher = create_publisher<waterlinked_dvl::msg::TransducerReportStamped>("dvl/transducer_report", 1000);
    m_transducer_report_publisher = create_publisher<waterlinked_dvl::msg::TransducerReportStamped>("dvl/transducer_distance_report", 1000);
    m_twist_publisher = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist", 1000);
    m_pose_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/pose", 1000);

    this->declare_parameter<std::vector<double>>("velocity_covariance");
    m_velocity_covariance = this->get_parameter("velocity_covariance").as_double_array();

    this->declare_parameter<std::vector<double>>("position_covariance");
    m_position_covariance = this->get_parameter("position_covariance").as_double_array();

    std::thread obj([this](){
        while(rclcpp::ok()) {
            this->f_serial_parse(this->f_serial_read());
        }
    });
    obj.detach();
}

void WaterlinkedDvlSerial::f_serial_parse(std::string incoming) {
    std::stringstream ss(incoming);
    std::string item;
    rclcpp::Time now = this->get_clock()->now();
    std::vector<std::string> elems;
    while(std::getline(ss, item, ',')) {
        elems.push_back(item);
    }

    if (!f_checksum(incoming)) {
        return;
    }

    if(elems.front() == "wrt") {

        waterlinked_dvl::msg::TransducerReportStamped report;
        report.header.stamp = now;
        report.header.frame_id = m_frame_id_;
        for(int i = 1 ; i < 5 ; i ++) {
            waterlinked_dvl::msg::Transducer t;
            t.id = i;
            t.distance = std::stof(elems[i]);
            report.report.transducers.push_back(t);
            m_transducers_publisher->publish(report);
        }

    } else if (elems.front() == "wrx") {
        waterlinked_dvl::msg::TransducerReportStamped report;
        report.header.stamp = now;
        report.header.frame_id = m_frame_id_;

        report.report.time_stamp = std::stof(elems[1]);
        report.report.vx = std::stof(elems[2]);
        report.report.vy = std::stof(elems[3]);
        report.report.vz = std::stof(elems[4]);
        report.report.fom = std::stof(elems[5]);
        report.report.altitude = std::stof(elems[6]);
        report.report.velocity_valid = elems[7] == "y";
        report.report.status = elems[8].at(0) == '1';

        m_transducer_report_publisher->publish(report);
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = report.header;
        twist_msg.twist.twist.linear.x = report.report.vx;
        twist_msg.twist.twist.linear.y = report.report.vy;
        twist_msg.twist.twist.linear.z = report.report.vz;

        if(m_velocity_covariance.size() == twist_msg.twist.covariance.size()) {
            std::array<double, 36> std_covariance;

            // Copy from boost::array to std::array
            std::copy(m_velocity_covariance.begin(), m_velocity_covariance.end(), std_covariance.begin());

            // Now assign to the message's covariance
            twist_msg.twist.covariance = std_covariance;
        }
        m_twist_publisher->publish(twist_msg);

    } if(elems.front() == "wrp") {
        waterlinked_dvl::msg::PositionReportStamped report;
        report.header.frame_id = m_frame_id_;
        report.header.stamp = now;
        report.report.time_stamp = std::stof(elems[1]);
        report.report.x = std::stof(elems[2]);
        report.report.y = std::stof(elems[3]);
        report.report.z = std::stof(elems[4]);
        report.report.std = std::stof(elems[5]);
        report.report.roll = std::stof(elems[6]);
        report.report.pitch = std::stof(elems[7]);
        report.report.yaw = std::stof(elems[8]);

        m_position_report_publisher->publish(report);


        tf2::Quaternion quaternion;
        quaternion.setRPY(report.report.roll, report.report.pitch, report.report.yaw);
        quaternion.normalize();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = report.header;
        pose_msg.pose.pose.position.x = report.report.x;
        pose_msg.pose.pose.position.y = report.report.y;
        pose_msg.pose.pose.position.z = report.report.z;

        pose_msg.pose.pose.orientation.w = quaternion.w();
        pose_msg.pose.pose.orientation.x = quaternion.x();
        pose_msg.pose.pose.orientation.y = quaternion.y();
        pose_msg.pose.pose.orientation.z = quaternion.z();

        if(m_position_covariance.size() == pose_msg.pose.covariance.size()) {
            std::array<double, 36> std_covariance;

            // Copy from boost::array to std::array
            std::copy(m_position_covariance.begin(), m_position_covariance.end(), std_covariance.begin());

            // Now assign to the message's covariance
            pose_msg.pose.covariance = std_covariance;
        }
        m_pose_publisher->publish(pose_msg);

    } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "ERROR");
    }

}

std::string WaterlinkedDvlSerial::f_serial_read() {
    std::string incoming;
    char c;
    for (;;) {
        boost::asio::read(m_ser_port, boost::asio::buffer(&c, 1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                return incoming;
            default:
                incoming+=c;
        }
    }
}

bool WaterlinkedDvlSerial::f_checksum(const std::string &incoming) {
    if(incoming.length() < 3) {
        return false;
    }

    auto inter = incoming.substr(0, incoming.size() - 3);
    auto data = reinterpret_cast<const uint8_t*>(inter.c_str());
    uint8_t crc = 0x00;
    size_t i, j;
    for (i = 0; i < inter.length(); i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                // polynomial is 0x7
                crc = (uint8_t)((crc << 1) ^ 0x7);
            else
                crc <<= 1;
        }
    }
    try {
        uint8_t incoming_crc = std::stoi(incoming.substr(incoming.length() - 2), 0, 16);
        return incoming_crc == crc;
    }
    catch (const std::invalid_argument& ia) {
        return false;
    }
}

void WaterlinkedDvlSerial::f_clean_up() {
    m_ser_port.close();
}

