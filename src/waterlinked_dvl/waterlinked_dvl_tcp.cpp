#include "waterlinked_dvl_tcp.h"
#include "json/json.h"
#include "algorithm"
#include "string"
#include "utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

WaterlinkedDvlTcp::WaterlinkedDvlTcp(rclcpp::NodeOptions options) :
        Node("waterlinked_dvl_node", options),        
        m_socket(m_io_service)
{
    // Declare parameters
    this->declare_parameter<std::string>("ip");
    this->declare_parameter<int>("port");
    this->declare_parameter<std::string>("frame_id");
    this->declare_parameter<std::vector<double>>("velocity_covariance");
    this->declare_parameter<std::vector<double>>("position_covariance");
    this->declare_parameter<int>("speed_of_sound", 1500);
    this->declare_parameter<int>("mounting_rotation_offset", 0);
    this->declare_parameter<bool>("acoustics_enabled", false);
    this->declare_parameter<std::string>("range_mode", "auto");
    this->declare_parameter<bool>("periodic_cycling_enabled", true);
    this->declare_parameter<int>("time_type", 0);

    // Get parameters
    this->get_parameter("ip", m_ip);
    this->get_parameter("port", m_port);
    this->get_parameter("frame_id", m_frame_id);
    this->get_parameter("velocity_covariance", m_velocity_covariance);
    this->get_parameter("position_covariance", m_position_covariance);
    this->get_parameter("speed_of_sound", m_speed_of_sound);
    this->get_parameter("mounting_rotation_offset", m_mounting_rotation_offset);
    this->get_parameter("acoustics_enabled", m_acoustic_enabled);
    this->get_parameter("range_mode", m_range_mode);
    this->get_parameter("periodic_cycling_enabled", m_periodic_cycling_enabled);
    this->get_parameter("time_type", m_time_type);
    RCLCPP_INFO(this->get_logger(), "%s", m_ip.c_str());
    

    // Create Publishers
    m_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist", 1000);
    m_raw_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/raw_twist", 1000);
    m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/pose", 1000);
    m_transducer_report_publisher = this->create_publisher<waterlinked_dvl::msg::TransducerReportStamped>("dvl/transducer_report", 1000);
    m_position_report_publisher = this->create_publisher<waterlinked_dvl::msg::PositionReportStamped>("dvl/position_report", 1000);
    m_altitude_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("dvl/altitude", 1000);

    //Socket connect
    boost::asio::ip::tcp::endpoint  endpoint(
            boost::asio::ip::address::from_string(m_ip), m_port);

    m_socket.connect(endpoint);

    boost::asio::socket_base::receive_buffer_size option(81920);
    m_socket.set_option(option);
    
    //Start a thread to read from DVL
    m_reading_thread = boost::thread(
        boost::bind(&WaterlinkedDvlTcp::f_read_loop, this)
    );
}

void WaterlinkedDvlTcp::f_read_loop() {
    boost::asio::streambuf incoming;
    while(rclcpp::ok()) {
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

    if (sent >= 0 && static_cast<std::string::size_type>(sent) == msg.size()) {
        return true;
    }
    return -1;
}

void WaterlinkedDvlTcp::f_parse_and_publish(std::string incoming) {


    Json::Value root;
    Json::Reader reader;

    try {
        reader.parse(incoming, root);

        auto format = root["format"].asString();

        if(format.find("v2") != std::string::npos) {
            f_parse_json_v2(root);
        }
        else if (format.find("v3") != std::string::npos) {
            f_parse_json_v3(root); 
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "Incoming message couldn't be parsed!: unknown format (" << format << ")");
        }

    } catch (Json::LogicError &e){
       RCLCPP_WARN_STREAM(this->get_logger(),"Incoming message couldn't be parsed!: " << e.what());
    }
}

void WaterlinkedDvlTcp::f_parse_json_v2(Json::Value root) {

    rclcpp::Time now = this->get_clock()->now();
    try{
        auto type = root["type"].asString();

        if(type == "velocity") {
            waterlinked_dvl::msg::TransducerReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;

            msg.report.time_stamp = root["time"].asFloat();
            msg.report.altitude = root["altitude"].asFloat();
            msg.report.status = root["status"].asInt();
            msg.report.fom = root["fom"].asFloat();
            msg.report.velocity_valid = root["velocity_valid"].asBool();
            msg.report.vx = root["vx"].asFloat();
            msg.report.vy = root["vy"].asFloat();
            msg.report.vz = root["vz"].asFloat();

            auto transducers_json = root["transducers"];
            for (Json::Value::ArrayIndex i = 0; i != transducers_json.size(); i++) {
                waterlinked_dvl::msg::Transducer transducer;

                transducer.distance = transducers_json[i]["distance"].asFloat();
                transducer.id = transducers_json[i]["id"].asInt();
                transducer.beam_valid = transducers_json[i]["beam_valid"].asBool();

                transducer.nsd = transducers_json[i]["nsd"].asFloat();
                transducer.rssi = transducers_json[i]["rssi"].asFloat();
                transducer.velocity = transducers_json[i]["velocity"].asFloat();

                msg.report.transducers.push_back(transducer);
            }

            m_transducer_report_publisher->publish(msg);

            if(msg.report.velocity_valid) {
                geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
                twist_msg.header = msg.header;
                twist_msg.twist.twist.linear.x = msg.report.vx;
                twist_msg.twist.twist.linear.y = msg.report.vy;
                twist_msg.twist.twist.linear.z = msg.report.vz;

                if(m_velocity_covariance.size() == twist_msg.twist.covariance.size()) {
                    std::array<double, 36> std_covariance;
        
                    // Copy from boost::array to std::array
                    std::copy(m_velocity_covariance.begin(), m_velocity_covariance.end(), std_covariance.begin());
        
                    // Now assign to the message's covariance
                    twist_msg.twist.covariance = std_covariance;
                }
                m_twist_publisher->publish(twist_msg);
            }

        } else if (type == "position_local") {
            waterlinked_dvl::msg::PositionReportStamped msg;
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
            msg.report.time_stamp = root["time"].asFloat();
            msg.report.std = root["std"].asFloat();

            m_position_report_publisher->publish(msg);

            tf2::Quaternion quaternion;
            quaternion.setRPY(msg.report.roll * TO_RADIAN, msg.report.pitch * TO_RADIAN, msg.report.yaw * TO_RADIAN);
            quaternion.normalize();

            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
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

            m_pose_publisher->publish(pose_msg);
        }
    } catch (Json::LogicError &e) {

    }

}

void WaterlinkedDvlTcp::f_parse_json_v3(Json::Value root){

    rclcpp::Time now = this->get_clock()->now();
    try
    {
        auto type = root["type"].asString();

        if(type == "velocity")
        {
            // ============================================================= //
            // parse all the data
            // ============================================================= //

            waterlinked_dvl::msg::TransducerReportStamped msg;
            msg.header.frame_id = m_frame_id;

            msg.report.vx = root["vx"].asFloat();
            msg.report.vy = root["vy"].asFloat();
            msg.report.vz = root["vz"].asFloat();
            msg.report.fom = root["fom"].asFloat();
            msg.report.time_stamp = root["time"].asFloat();

            int index = 0;
            for(Json::Value::ArrayIndex i = 0 ;  i != root["covariance"].size() ; i++) {
                for(Json::Value::ArrayIndex j = 0 ; j != root["covariance"][i].size() ; j ++) {
                    msg.report.covariance[index++] = root["covariance"][i][j].asFloat();
                }
            }

            msg.report.altitude = root["altitude"].asFloat();
            
            auto transducers_json = root["transducers"];
            for (Json::Value::ArrayIndex i = 0; i != transducers_json.size(); i++) {
                waterlinked_dvl::msg::Transducer transducer;

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


            uint64_t time_in_nanos_validity = root["time_of_validity"].asUInt64() * 1000U;  // Convert microseconds to nanoseconds
            int64_t seconds_val = time_in_nanos_validity / 1e9;
            int64_t nanoseconds_val = time_in_nanos_validity % static_cast<int64_t>(1e9);
            // Create a rclcpp::Duration from nanoseconds
            rclcpp::Duration duration_time_validity(seconds_val, nanoseconds_val
            );   // Construct a duration directly from nanoseconds
            msg.report.time_of_validity.sec = duration_time_validity.seconds();
            msg.report.time_of_validity.nanosec = duration_time_validity.nanoseconds();  

            
           
            uint64_t time_in_nanos_transmission = root["time_of_transmission"].asUInt64() * 1000U;  // Convert microseconds to nanoseconds
            int64_t seconds_trans = time_in_nanos_transmission / 1e9;
            int64_t nanoseconds_trans = time_in_nanos_transmission % static_cast<int64_t>(1e9);
            // Create a rclcpp::Duration from nanoseconds
            rclcpp::Duration duration_time_transmission(seconds_trans, nanoseconds_trans);   // Construct a duration directly from nanoseconds
            msg.report.time_of_transmission.sec = duration_time_transmission.seconds();
            msg.report.time_of_transmission.nanosec = duration_time_transmission.nanoseconds();  

            //! DEBUG:
            // printf("ros now: %.9f, validity:%.9f, trans:%.9f\n", 
            //     now.toSec(), 
            //     msg.report.time_of_validity.toSec(), 
            //     msg.report.time_of_transmission.toSec());

            // ============================================================= //
            // setup the time type
            // ============================================================= //

            if(m_time_type == 0) {
                msg.header.stamp = now;
            }
            if (m_time_type == 1) {
                // Convert time_of_transmission and now to seconds
                double time_of_transmission_in_seconds = timeToSeconds(msg.report.time_of_transmission);
                double now_in_seconds = now.seconds();
            
                if (time_of_transmission_in_seconds - now_in_seconds > 5.0) {
                    RCLCPP_ERROR(this->get_logger(), "DVL is not time synchronized, do not use this time_type!");
                }
                
                // Set the header timestamp to time_of_validity
                msg.header.stamp = msg.report.time_of_validity;
            }
            else if (m_time_type == 2) {
                // Convert time_of_transmission and now to seconds
                double time_of_transmission_in_seconds = timeToSeconds(msg.report.time_of_transmission);
                double now_in_seconds = now.seconds();
            
                if (time_of_transmission_in_seconds - now_in_seconds > 5.0) {
                    RCLCPP_ERROR(this->get_logger(), "DVL is not time synchronized, do not use this time_type!");
                }
                
                // Set the header timestamp to time_of_transmission
                msg.header.stamp = msg.report.time_of_transmission;
            }

            // ============================================================= //
            // publish the msg
            // ============================================================= //

            //Raw Twist
            geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
            twist_msg.header = msg.header;
            twist_msg.twist.twist.linear.x = msg.report.vx;
            twist_msg.twist.twist.linear.y = msg.report.vy;
            twist_msg.twist.twist.linear.z = msg.report.vz;

            //Twist Covariance holds in its diagnol: u,v,w,p,q,r.
            //Msg report covariance is in u,v,w (3x3)
            twist_msg.twist.covariance[0] = msg.report.covariance[0];
            twist_msg.twist.covariance[1] = msg.report.covariance[1];
            twist_msg.twist.covariance[2] = msg.report.covariance[2];
            twist_msg.twist.covariance[6] = msg.report.covariance[3];
            twist_msg.twist.covariance[7] = msg.report.covariance[4];
            twist_msg.twist.covariance[8] = msg.report.covariance[5];
            twist_msg.twist.covariance[12] = msg.report.covariance[6];
            twist_msg.twist.covariance[13] = msg.report.covariance[7];
            twist_msg.twist.covariance[14] = msg.report.covariance[8];

            //Publish the raw twist msg
            m_raw_twist_publisher->publish(twist_msg);

            if(msg.report.velocity_valid) {
                //Publish the valid twist msg
                m_twist_publisher->publish(twist_msg);
            }

            //Publish the altitude msg
            geometry_msgs::msg::PointStamped alt_msg;
            if(msg.report.altitude>0)
            {
                alt_msg.header = msg.header;
                alt_msg.point.x = 0;
                alt_msg.point.y = 0;
                alt_msg.point.z = msg.report.altitude;
                m_altitude_publisher->publish(alt_msg);
            }

            //Publish the report msg
            m_transducer_report_publisher->publish(msg);            
        }
        else if (type == "position_local")
        {
            waterlinked_dvl::msg::PositionReportStamped msg;
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
            msg.report.time_stamp = root["ts"].asFloat();
            msg.report.std = root["std"].asFloat();

            msg.report.format = root["format"].asString();

            m_position_report_publisher->publish(msg);

            tf2::Quaternion quaternion;
            quaternion.setRPY(msg.report.roll * TO_RADIAN, msg.report.pitch * TO_RADIAN, msg.report.yaw * TO_RADIAN);
            quaternion.normalize();

            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
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

            m_pose_publisher->publish(pose_msg);
        }
    
    } catch (Json::LogicError &e) {

    }
}

double WaterlinkedDvlTcp::timeToSeconds(const builtin_interfaces::msg::Time& time_msg) {
    return time_msg.sec + time_msg.nanosec * 1e-9;
}