/*获取imu数据，并以sensor_msgs/IMU类型的msg发布出去
 *发布的topic名称为imu_data;
 *坐标系应该是odom->base_footprint,这里大概会要计算一下一个变换？
 */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <string> 

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_get");
    ros::NodeHandle nh;

    return 0;
}