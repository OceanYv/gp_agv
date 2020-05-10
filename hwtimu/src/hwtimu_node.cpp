#include <ros/ros.h>
#include "hwtimu/hwtimu.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"hwtimu_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
    ROS_INFO("----------Begin of the hwtimu_node!----------");
    Hwtimu myHwtimu(nh,nh_private);
	ros::spin();
    return 0;
}

