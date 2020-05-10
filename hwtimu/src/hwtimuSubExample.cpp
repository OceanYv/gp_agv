#include <ros/ros.h>
//#include <hwtimu/hwt901imudata.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#define WYC_PI ((double)3.141592653589793)

void imudataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    /*
    * roll=atan2(2(wx+yz),1-2(x^2+y^2))
    * pitch=arcsin(2(wy-zx))
    * yaw=atan2(2(wz+xy),1-2(y^2+z^2))
    */

    // roll (x-axis rotation)
    double roll,pitch,yaw;
    double sinr = +2.0*(msg->orientation.w*msg->orientation.x + msg->orientation.y*msg->orientation.z);
    double cosr = +1.0-2.0*(msg->orientation.x*msg->orientation.x+msg->orientation.y*msg->orientation.y);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0*(msg->orientation.w*msg->orientation.y-msg->orientation.z*msg->orientation.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(WYC_PI/2.0, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0*(msg->orientation.w*msg->orientation.z+msg->orientation.x*msg->orientation.y);
    double cosy = +1.0-2.0*(msg->orientation.y*msg->orientation.y+msg->orientation.z*msg->orientation.z);  
    yaw = atan2(siny, cosy);

    // radian to degree
    roll=roll/WYC_PI*180.0;
    pitch=pitch/WYC_PI*180.0;
    yaw=yaw/WYC_PI*180.0;

    //add according to AHRSIMU
    if(yaw<-180.0) yaw=180.0-(-180.0-yaw);
    else if(yaw>180){
        yaw=-180.0-(180.0-yaw);
    }

    ROS_INFO("----------Function imudataCallback!----------");
    ROS_INFO("accl[x,y,z]=[%6.4f,%6.4f,%6.4f]",
              msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
    ROS_INFO("gyro[x,y,z]=[%6.4f,%6.4f,%6.4f]",
              msg->angular_velocity.x,
              msg->angular_velocity.y,
              msg->angular_velocity.z);
    ROS_INFO("angl[x,y,z]=[%6.4f,%6.4f,%6.4f]",
              roll,
              pitch,
              yaw);
    ROS_INFO("qutn[x,y,z,w]=[%6.4f,%6.4f,%6.4f,%6.4f]",
              msg->orientation.x,
              msg->orientation.y,
              msg->orientation.z,
              msg->orientation.w);
    ROS_INFO("----------End of callback function!----------");
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"hwtimuSubExample");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string subscribe_topic_="imu/data_raw";
    nh_private.param("subscribe_topic",subscribe_topic_,std::string("imu/data_raw"));
    ros::Subscriber sub=nh.subscribe(subscribe_topic_,1,imudataCallback);
    ros::spin();
    return 0;
}
