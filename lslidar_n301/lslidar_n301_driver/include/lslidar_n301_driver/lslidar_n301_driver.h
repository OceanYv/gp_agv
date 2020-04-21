/*
 * This file is part of lslidar_n301 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_N301_DRIVER_H
#define LSLIDAR_N301_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_n301_msgs/LslidarN301Packet.h>

namespace lslidar_n301_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LslidarN301Driver {
public:

    LslidarN301Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarN301Driver();

    bool initialize();
    bool polling();

    typedef boost::shared_ptr<LslidarN301Driver> LslidarN301DriverPtr;
    typedef boost::shared_ptr<const LslidarN301Driver> LslidarN301DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_n301_msgs::LslidarN301PacketPtr& msg);

    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;

    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;
};

typedef LslidarN301Driver::LslidarN301DriverPtr LslidarN301DriverPtr;
typedef LslidarN301Driver::LslidarN301DriverConstPtr LslidarN301DriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_N301_DRIVER_H_
