/*该文件目前无用*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <base_controller/Sensorfusion.h>


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "sensor_fusion");
	ros::NodeHandle nh;

	FUSION::Sensorfusion my_fusioner(nh);  //这个类定义在Sensorfusion.h中
	ros::spin();

	return 0;
}