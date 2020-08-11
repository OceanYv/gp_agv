//该文件目前无用
#ifndef __LOCAL_EKF__
#define __LOCAL_EKF__

#include <string>
#include <math.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>			//用于里程计
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>	//用于tf
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float32.h>

using namespace Eigen;

namespace FUSION{

/*这个类实现了的单个传感器的局部测量更新
 *每次更新需要的输入：预测位姿均值及其协方差、传感器获得的已经转换坐标的位姿（测量）及其测量不确度的协方差矩阵
 */
class localEKF
{
public:
	/*构造函数
     *1.初始化传感器基本信息
     *1.5.创建数据订阅者，确定名字
     *2.初始化传感器各种计算的矩阵
     *3.初始化各种运行状态的参数
     *4.完成构造之后，还要相应的修改状态参数；
     */
	localEKF(ros::NodeHandle nh, std::string name);
	~localEKF();
    
    /*处理获取信息的回调函数，主要是处理订阅的传感器数据（包括测量值、协方差和时刻）
     *根据不同的输入类型，执行不同的函数；
     *将数据接收到接收并通过坐标转换到base_footprint坐标系上，并储存在last_meas_，之后将have_data_置true；
     */
    void odom_local_callback(const nav_msgs::Odometry &odom);
    void imu_local_callback(const sensor_msgs::Imu &imu);

    /*更新函数,由sysup_callback调用
     *输入系统更新得到的参数，通过last_meas_cov_和last_meas_pose_计算局部测量更新参数
     */
    void update(Vector3f pose_sysup,Matrix3f cov_sysup);

    /*通过终端输出滤波器状态
     *暂时用不到，可以先不写（空函数）
     */
    void getstatus();

    bool have_data_;    //是否有已经获取数据的标识
    bool sensor_initialized_;
    nav_msgs::Odometry last_meas_;	   //最新通过订阅获得的传感器数据（要记得变换到odom这个坐标上）
    Vector3f last_meas_pose_;    //局部测量更新值；
    Matrix3f k_,last_meas_cov_;   //卡尔曼增益,协方差矩阵
    ros::Time now_stamp_, last_stamp_;
private:
    //基本配置参数
    ros::NodeHandle node_handle_;
    std::string sensor_name_;    /*传感器的名字*/
    
    //传感器原始数据订阅、获取的数据、完成局部更新的数据
    ros::Subscriber sub_;
    tf::TransformListener robot_state_;  //tf接收者

    //计算过程的参数
    Vector3f last_pose_t0_;    //直接从传感器得到的；
    Matrix3f last_cov_t0_; 
    double trans_x_;    //IMU在小车上的安装关系
    Matrix3f Identity_matrix_;

    //计算IMU积分的一些参数
    double angularvel_t0_,angularvel_t1_,linearacc_t0_,linearacc_t1_,linevel_; //角速度，线加速度，线速度
	
};//end of the class

};//end of the namespace
#endif