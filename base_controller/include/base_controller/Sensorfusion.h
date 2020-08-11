//该文件目前无用
#ifndef __SENSOR_FUSION__
#define __SENSOR_FUSION__

#include "base_controller/localEKF.h"

namespace FUSION{

class Sensorfusion
{
public:
    /*
     *构建函数
     *完成的工作：
        1.通过参数服务器读取预设的参数(传感器是否使用、)
        2.创建定时器，指定频率和回调函数
        3.创建topic发布者、根据配置创建传感器数据订阅者，指定相应的回调函数（就是，变量定义）
        4.调用sys_init函数初始化系统滤波器
     */
	Sensorfusion(ros::NodeHandle nh);

    /*析构函数
     *删除定义的类的对象
     */
	~Sensorfusion();

    /*系统滤波回调函数
     *输入这个时间段内的控制;
     *通过fusion_pub_计算得到sys_pred_;

     *调用各个传感器的update函数，进行局部测量；
     *判断获取的参数是否有异常（主要是系统数据与两个传感器数据在时间上是否有较大误差）
     *计算各个传感器数据间的交叉协方差，然后计算权重矩阵（主要是pij的计算，用理论2的推论1实现）
        在odom与IMU数据相差较大的时，将IMU结果作为最终结果（IMU的权重为1，ODOM的权重为1），还要warning一下;
     *调用do_fusion完成融合（利用理论1）
     *发布相应的topic（融合定位结果、时刻、协方差矩阵）和tf
     */
    void sysup_callback(const geometry_msgs::Twist &cmd_input);

private:
    //局部更新滤波器（这个类定义在local_ekf.h文件）
    FUSION::localEKF* imu_ekf_;
    FUSION::localEKF* odom_ekf_;
    //MatrixWrapper::SymmetricMatrix odom_covaricance_,imu_covaricance_;

    ros::NodeHandle node_handle_;

    //状态参数，设置参数
    bool odom_used_,imu_used_;
    bool odom_actived_,imu_actived_,sys_initialized_;
    double imu_rate_,odom_rate_,update_rate_;
    
    //用于发布、订阅的参数
    ros::Publisher pose_pub_;   //融合结果的发布
    tf::TransformBroadcaster odom_bc_;  //tf tree的发布
    ros::Subscriber get_cmdvel_;//期望速度的订阅
    double cmdvel_[2];   //速度控制
    geometry_msgs::TransformStamped odom_tf_;   //用于发布tf转换的变量
    
    nav_msgs::Odometry fusion_pub_;      //融合得到的结果（初始值也存在这里面）
    nav_msgs::Odometry sys_pred_;        //系统预测的结果

    //一些过程变量
    ros::Time now_stamp_, last_stamp_;
    Vector3f pose_t0_, pose_t1_;    //表示位姿(x,y,θ)
    Matrix3f cov_t0_, cov_t1_;      //方便计算的协方差矩阵
    Matrix3f Gt_, Rt_;             //误差传递矩阵，运动白噪声协方差矩阵；

    Matrix3f Identity_matrix_,Pij_;  //单位矩阵，两个传感器的交叉协方差
    Matrix3f weight_imu_,weight_odom_;
    Matrix<float, 6, 3> E63_,A63_; //两个单位矩阵组成的矩阵,储存了两个传感器数据的权重；
    Matrix<float, 6, 6> P66_,P66_inverse;   //一个过程变量罢了

};
};//end of the namespace FUSION

#endif

/*数据融合程序中噪声与协方差矩阵传递过程
1.里程计数据的不确定度在base_controller中赋予
  IMU数据的不确定度在IMU对应的local_callback中赋予  ->  要定义一个私有成员变量单独来存用到的三个值，来方便计算（3*3矩阵）
2.初值的不确定度在通过IMU数据获取的时候赋值
  系统预测更新中，通过fusion_pub_的不确定度、以及sysup_callback中定义的运动噪声，得到sys_pred_中的不确定度；    -> 运动噪声要定义一个协方差矩阵的
3.传感器测量噪声在update中引入         -> 传感器噪声要定义一个协方差矩阵的
  融合结果的协方差矩阵在sysup_callback计算得到
*/