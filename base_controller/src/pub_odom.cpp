#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class pub_odomc
{
    public:
        pub_odomc(){
            pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);  
            sub_ = nh_.subscribe("odom_combined", 10, &pub_odomc::odom_Callback,this);//使用类内函数的时候用this
        }
        void odom_Callback(const geometry_msgs::PoseWithCovarianceStamped &odom_comb){  
            nav_msgs::Odometry odom_inf;
            odom_inf.header=odom_comb.header;
            odom_inf.child_frame_id = "base_footprint";
            odom_inf.pose=odom_comb.pose;
//          odom_inf.twist.twist.linear = ;
//          odom_inf.twist.twist.angular = ;
//          odom_inf.twist.covariance = ;
            pub_.publish(odom_inf);  
        }
      
    private:
      ros::NodeHandle nh_;   
      ros::Publisher pub_;  
      ros::Subscriber sub_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "pub_odom");
    pub_odomc pubobject;
    ros::spin();
    return 0;
}