#include "base_controller/localEKF.h"

namespace FUSION {

localEKF::localEKF(ros::NodeHandle nh,std::string name):
    have_data_(false),
    node_handle_(nh),
    sensor_name_(name),
    sensor_initialized_(false),
    linevel_(0)
{  
    //创建订阅者
    std::string odom,imu;
    odom="ODOM",imu="IMU";
    if(name==odom)
        sub_ = node_handle_.subscribe("odom_data", 2, &localEKF::odom_local_callback, this);
    else if(name==imu)
        sub_ = node_handle_.subscribe("imu/data_raw", 2, &localEKF::imu_local_callback, this);
    else
        ROS_ERROR("Unkonwn sensor name");
    //初始化各种变量
    ros::Time now_stamp=ros::Time::now();
    last_meas_.header.stamp = now_stamp; 
    last_meas_.header.frame_id = "odom";  //位置是在odom坐标系下的
    last_meas_.child_frame_id = "odom"; //速度是在odom坐标系下的
    last_meas_.pose.pose.position.x= last_meas_.pose.pose.position.y= last_meas_.pose.pose.position.z= 0;
    last_meas_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    float covariance[36] = {0.01,   0,  0,  0,  0,  0,  //位置和速度的测量不确定性协方差矩阵
                            0,  0.01,   0,  0,  0,  0,
                            0,  0,  99999,  0,  0,  0,
                            0,  0,  0,  99999,  0,  0,
                            0,  0,  0,  0,  99999,  0,
                            0,  0,  0,  0,  0,  0.01};
    for(int i = 0; i < 36; i++)
        last_meas_.pose.covariance[i] = covariance[i];

    last_pose_t0_ << 0,0,0;
    last_meas_pose_ << 0,0,0;
    last_cov_t0_ << 0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01;
    angularvel_t0_=angularvel_t1_=linearacc_t0_=linearacc_t1_=0;
    now_stamp_ = last_stamp_ = now_stamp;
    Identity_matrix_ = Matrix3f::Identity(3,3);
    
    node_handle_.param("fixed_tf/base_imu_tf/x",trans_x_,0.0);
    //完成初始化
    sensor_initialized_ = true;
}

localEKF::~localEKF(){};

void localEKF::odom_local_callback(const nav_msgs::Odometry &odom){
    //判断是否完成初始化
    ros::Rate loop_rate(100);
    for(int i=0;i<5;i++){
        if(sensor_initialized_ == false){
            if(i==4) ROS_ERROR("The odom callback function is called befor init is finished!");
            loop_rate.sleep();
        }
        else
            break;
    }

    //获取last_meas_,last_pose_t0_,last_cov_t0_
    last_meas_ = odom;
    double w_t0 = last_meas_.pose.pose.orientation.w;
    double z_t0 = last_meas_.pose.pose.orientation.z;
    last_pose_t0_[0] = last_meas_.pose.pose.position.x;
    last_pose_t0_[1] = last_meas_.pose.pose.position.y;
    last_pose_t0_[2] = atan2(2*w_t0*z_t0,1-2*z_t0*z_t0);
    double cov[36];
    for(int i=0;i<36;i++){
        cov[i] = last_meas_.pose.covariance[i];
    }
    
    last_cov_t0_ << cov[0],cov[1],cov[5], cov[6],cov[7],cov[11], cov[30],cov[31],cov[35];
    now_stamp_ = last_meas_.header.stamp;

    have_data_ = true;
}

void localEKF::imu_local_callback(const sensor_msgs::Imu &imu){
    //判断是否完成初始化
    ros::Rate loop_rate(100);
    for(int i=0;i<5;i++){
        if(sensor_initialized_ == false){
            if(i==4) ROS_ERROR("The imu callback function is called befor init is finished!");
            loop_rate.sleep();
        }
        else
            break;
    }
    //数据获取
    now_stamp_ = last_meas_.header.stamp = imu.header.stamp;
    double dt = (now_stamp_ - last_stamp_).toSec();     //时间变化
    angularvel_t1_ = imu.angular_velocity.z;
    linearacc_t1_ = imu.linear_acceleration.x;
    linearacc_t1_ += angularvel_t1_*angularvel_t1_*trans_x_;
    double angularvel_mean_ = (angularvel_t0_ +angularvel_t1_)/2;  //角速度平均值
    double linearacc_mean_ = (linearacc_t0_+linearacc_t1_)/2;  //线加速度平均值
    //计算测量值
    linevel_ += linearacc_mean_*dt;
    last_pose_t0_[0] += (linevel_ + 0.5*linearacc_mean_*dt)*dt*cos(last_pose_t0_[2]+0.5*angularvel_mean_*dt);
    last_pose_t0_[1] += (linevel_ + 0.5*linearacc_mean_*dt)*dt*sin(last_pose_t0_[2]+0.5*angularvel_mean_*dt);
    last_pose_t0_[2] += angularvel_mean_*dt;

    last_meas_.header.frame_id = "odom";
    last_meas_.child_frame_id = "odom";
    last_meas_.pose.pose.position.x = last_pose_t0_[0];
    last_meas_.pose.pose.position.y = last_pose_t0_[1];
    last_meas_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(last_pose_t0_[2]);

    angularvel_t0_=angularvel_t1_;
    linearacc_t0_=linearacc_t1_;
    have_data_ = true;
}

void localEKF::update(Vector3f pose_sysup, Matrix3f cov_sysup){

    k_ = cov_sysup * ((cov_sysup + last_cov_t0_).inverse());
    last_meas_pose_ = pose_sysup + k_ * (last_pose_t0_ - pose_sysup);
    last_meas_cov_ = (Identity_matrix_ - k_) * cov_sysup;

    last_pose_t0_ = pose_sysup;
    last_cov_t0_ = cov_sysup;
}

void localEKF::getstatus(){

}

}//end of the namepace