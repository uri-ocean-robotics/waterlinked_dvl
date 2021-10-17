#include "waterlinked_dvl_serial.h"
#include "ros/console.h"
#include "boost/crc.hpp"
#include "thread"


WaterlinkedDvlSerial::WaterlinkedDvlSerial()
    :
        m_pnh("~"),
        m_nh(),
        m_io(),
        m_ser_port(m_io)  {

    std::string port;
    m_pnh.param<std::string>("port", port, "/dev/ttyUSB0");
    m_pnh.param<std::string>("frame_id", m_frame_id, "dvl_link");

    m_ser_port.open(port);
    
    m_ser_port.set_option(boost::asio::serial_port_base::baud_rate(115200));


    m_transducer_report_publisher = m_nh.advertise<waterlinked_dvl::TransducerReportStamped>("transducer_report", 1000);
    m_transducers_publisher = m_nh.advertise<waterlinked_dvl::TransducerReportStamped>("transducer_distance_report", 1000);
    m_position_report_publisher = m_nh.advertise<waterlinked_dvl::PositionReportStamped>("position_report", 1000);


    std::thread obj([this](){
        while(ros::ok()) {
            this->f_serial_parse(this->f_serial_read());
        }
    });
    obj.detach();
}

void WaterlinkedDvlSerial::f_serial_parse(std::string incoming) {
    std::stringstream ss(incoming);
    std::string item;
    std::vector<std::string> elems;
    while(std::getline(ss, item, ',')) {
        elems.push_back(item);
    }

    if (!f_checksum(incoming)) {
        return;
    }

    if(elems.front() == "wrt") {

        waterlinked_dvl::TransducerReportStamped report;
        report.header.stamp = ros::Time::now();
        report.header.frame_id = m_frame_id;
        for(int i = 1 ; i < 5 ; i ++) {
            waterlinked_dvl::Transducer t;
            t.id = i;
            t.distance = std::stof(elems[i]);
            report.report.transducers.push_back(t);
            m_transducers_publisher.publish(report);
        }

    } else if (elems.front() == "wrx") {
        waterlinked_dvl::TransducerReportStamped report;
        report.header.stamp = ros::Time::now();
        report.header.frame_id = m_frame_id;

        report.report.time = std::stof(elems[1]);
        report.report.vx = std::stof(elems[2]);
        report.report.vy = std::stof(elems[3]);
        report.report.vz = std::stof(elems[4]);
        report.report.fom = std::stof(elems[5]);
        report.report.altitude = std::stof(elems[6]);
        report.report.velocity_valid = elems[7] == "y";
        report.report.status = elems[8].at(0) == '1';

        m_transducer_report_publisher.publish(report);

    } if(elems.front() == "wrp") {
        waterlinked_dvl::PositionReportStamped report;
        report.header.frame_id = m_frame_id;
        report.header.stamp = ros::Time::now();
        report.report.time = std::stof(elems[1]);
        report.report.x = std::stof(elems[2]);
        report.report.y = std::stof(elems[3]);
        report.report.z = std::stof(elems[4]);
        report.report.std = std::stof(elems[5]);
        report.report.roll = std::stof(elems[6]);
        report.report.pitch = std::stof(elems[7]);
        report.report.yaw = std::stof(elems[8]);

        m_position_report_publisher.publish(report);

    } else {
        ROS_WARN_STREAM("Error");
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

