/*该文件目前无用*/
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>			//用于里程计
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>	//用于tf
 
class Imu_preinte
{
public:
  Imu_preinte()
  {
    //读取参数服务器中IMU安装的数据
    n_.param("/imu/x",x_,0.2);
    n_.param("/imu/y",y_,0.0);
    n_.param("/imu/z",z_,0.0);
    //定义接受者、发送者与服务者
    pub_ = n_.advertise<nav_msgs::Odometry>("odom_imu", 1);
    imu_sub_ = n_.subscribe("imu/data_raw", 1, &Imu_preinte::callback1, this);
    odom_sub_ = n_.subscribe("odom", 1, &Imu_preinte::callback2, this);
    ser_ = n_.advertiseService("imu_inte_adj", &Imu_preinte::handle_function, this);
    //初始化imu里程计数据
    odomimu_.header.stamp = ros::Time::now();
    odomimu_.header.frame_id = "map";
    odomimu_.child_frame_id = "base_footprint";
    odomimu_.pose.pose.position.x = odomimu_.pose.pose.position.y = odomimu_.pose.pose.position.z = 0;
    odomimu_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    float covariance[36] = {10,   0,  0,  0,  0,  0,
                            0,  10,   0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,  10};
    for(int i = 0; i < 36; i++)
        odomimu_.pose.covariance[i] = covariance[i];

    vx_imu_ = thet_imu_ = 0;
    odomadj_flag_ = false;
  }

  ~Imu_preinte(){};
 
  void callback1(const sensor_msgs::Imu &imu)
  {
    double dt = ( imu.header.stamp - odomimu_.header.stamp).toSec();     //时间变化
    odomimu_.header.stamp = imu.header.stamp;

    vx_imu_ = (vx_imu_ + dt * imu.linear_acceleration.x)/2;
    odomimu_.twist.twist.angular.z = w_imu_ = (w_imu_ + imu.angular_velocity.z)/2;
    odomimu_.twist.twist.linear.x = vx_imu_ + y_ * w_imu_;

    thet_imu_ += w_imu_ * dt;
    odomimu_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(thet_imu_);
    odomimu_.pose.pose.position.x += vx_imu_ * dt * cos(thet_imu_);
    odomimu_.pose.pose.position.y += vx_imu_ * dt * sin(thet_imu_);
        
    pub_.publish(odomimu_);
  }

  void callback2(const nav_msgs::Odometry &odom)
  {
    if(odomadj_flag_ == false)
      return;

    /*读取odom来更新的程序*/
    odomadj_flag_ = false;
  }

  bool handle_function(std_srvs::Empty::Request &input,std_srvs::Empty::Response &output)
  {
    odomadj_flag_ = true;
    return true;
  }
 
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber imu_sub_,odom_sub_;
  ros::ServiceServer ser_;

  nav_msgs::Odometry odomimu_;
  double vx_imu_, w_imu_, thet_imu_;

  double x_,y_,z_;      //IMU安装位置

  bool odomadj_flag_;
};


 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_pre_inte");
  Imu_preinte imu_pre;
  ros::spin();
  return 0;
}