#ifndef WATERLINKED_DVL_WATERLINKED_DVL_H
#define WATERLINKED_DVL_WATERLINKED_DVL_H

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "thread"

class WaterlinkedDvl {
private:

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    boost::asio::io_context m_io_context;
    boost::asio::ip::tcp::socket m_socket;


    std::string m_ip;
    int m_port;

    void f_readloop();

    void f_parse_and_publish(std::string incoming);

    boost::thread m_reading_thread;

    ros::Publisher m_transducer_publisher;

    ros::Publisher m_position_publisher;

public:

    WaterlinkedDvl();

};


#endif //WATERLINKED_DVL_WATERLINKED_DVL_H
