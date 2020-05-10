#ifndef HWTIMU_H_
#define HWTIMU_H_

#include <ros/ros.h>
#include <termios.h>
#include <string>
#include <boost/thread.hpp>
#include "hwtimu/com.h"

class Hwtimu
{
public:
    Hwtimu();
	Hwtimu(ros::NodeHandle nh,ros::NodeHandle nh_private);
    ~Hwtimu();
    ros::NodeHandle node_handle_;
	ros::NodeHandle node_handle_pri_;
    ros::Publisher hwtimu_data_pub_;

    std::string com_name_;
    int baud_rate_;
    int data_bits_;
    int parity_;
    int stop_bits_;
    int mini_data_;

    std::string frame_id_;
    bool burst_mode_;
    double loop_rate_;
    std::string imu_topic_;
    //std::string com_file_;
    int kind_of_imu_;
    bool is_debug_;

    CCom HwtimuCom;
    boost::thread* imuRcvThread;

    // Lineal Acceleration sensor(x, y, z)
    double accl[3];
    // Gyro sensor(x, y, z)(Rotational velocity)
    double gyro[3];
    // magnetic sensor(x,y,z)
    double magn[3];
    // Angle sensor(x, y, z)
    double angl[3];
    // orientation(q0, q1, q2, q3)
    double orten[4];

    int publish_imu_data();
    int sampleFromPort();
    int sampleFromPort_lpms();
    int resolveSingleCmd(unsigned char* data);
    int resolveSingleCmd_lpms(unsigned char* data,size_t &length);
    bool sumCheck(unsigned char* data);
    bool sumCheck_lpms(unsigned char* data);
    int truncateData(unsigned char* dataBuffer,size_t &length);
    int truncateData_lpms(unsigned char* dataBuffer,size_t &length);
    int16_t big_endian_to_short(unsigned char *data);
    int16_t little_endian_to_short(unsigned char *data);
    double little_endian_to_double(unsigned char *data);
    double little_endian_to_double_lpms(unsigned char *data);
    int print_data_buffer(unsigned char* data,int len,int printColor);

    int openPort();
    void closePort();
    int update(void);
    int update_burst(void);
    int read_register(char address, int16_t& data);
    int get_product_id(std::string& data);
};

#endif

