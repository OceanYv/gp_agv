#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <unistd.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "acml_global_init");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("global_localization");
    std_srvs::Empty srv;

    int trytimes = 0;
    while(!(client.call(srv)) && trytimes < 5){
        trytimes++;
        sleep(0.5);
    }

    if(trytimes == 5)
        ROS_ERROR("Fail to call global localization");
    else
        ROS_INFO("Call global localization successfully");

    return 1;
}
