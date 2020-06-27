#include "base_controller/Sensorfusion.h"

namespace FUSION{

Sensorfusion::Sensorfusion(ros::NodeHandle nh)
    :odom_used_(false),
    imu_used_(false),
    update_rate_(0.0),
    odom_actived_(false),
    imu_actived_(false),
    sys_initialized_(false),
    node_handle_(nh)
{
    node_handle_.param("sensor_fusion/odom_used", odom_used_, false);
    node_handle_.param("sensor_fusion/imu_used", imu_used_, false);
    node_handle_.param("sensor_fusion/update_rate", update_rate_, 10.0);

    ros::Time now_stamp;

    //融合数据的发布者、tf_tree发布者、cmd_vel的订阅者、两个传感器
    pose_pub_ = node_handle_.advertise<nav_msgs::Odometry>("odom", 2);

    get_cmdvel_ = node_handle_.subscribe("cmd_vel", 2, &Sensorfusion::sysup_callback , this);

    if((odom_used_ || imu_used_) == false)
      ROS_ERROR("No sensor has been used!!!");

    if (odom_used_){
      ROS_INFO("ODOM is used!");
      odom_ekf_ = new localEKF(node_handle_,"ODOM");
      odom_actived_ = true;
    }
    //else ROS_DEBUG("没有使用Odom");
    if (imu_used_){
      ROS_INFO("IMU is used!");
      imu_ekf_ = new localEKF(node_handle_,"IMU");
      imu_actived_ = true;
    }
    //else ROS_DEBUG("没有使用Imu");

    //初始化计算要用到的值
    sys_pred_.header.stamp = now_stamp; 
    sys_pred_.header.frame_id = "odom";  //位置是在odom坐标系下的
    sys_pred_.child_frame_id = "odom"; //速度是在odom坐标系下的
    sys_pred_.pose.pose.position.x= sys_pred_.pose.pose.position.y= sys_pred_.pose.pose.position.z= 0;
    sys_pred_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    float covariance[36] = {0.01,   0,  0,  0,  0,  0,  //位置和速度的测量不确定性协方差矩阵
                            0,  0.01,   0,  0,  0,  0,
                            0,  0,  99999,  0,  0,  0,
                            0,  0,  0,  99999,  0,  0,
                            0,  0,  0,  0,  99999,  0,
                            0,  0,  0,  0,  0,  0.01};
    for(int i = 0; i < 36; i++)
        sys_pred_.pose.covariance[i] = covariance[i];
    
    fusion_pub_ = sys_pred_;

    cmdvel_[0]=cmdvel_[1]=0;
    pose_t0_ = Vector3f::Zero();
    pose_t1_ = Vector3f::Zero();
    last_stamp_=now_stamp_=ros::Time::now();

    pose_t0_[1] = 0;
    pose_t0_[2] = 0;
    cov_t0_ << 0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01;
    Rt_ << 0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01; 

    Identity_matrix_ = Matrix3f::Identity(3,3);
    E63_.block<3,3>(0,0) << Identity_matrix_;
    E63_.block<3,3>(3,0) << Identity_matrix_;
    Pij_ << 0.01,0,0, 0,0.01,0, 0,0,0.01;
};

Sensorfusion::~Sensorfusion(){
    delete imu_ekf_;
    delete odom_ekf_;
}

void Sensorfusion::sysup_callback(const geometry_msgs::Twist &cmd_input){
  //判断是否完成初始化
  ros::Rate loop_rate(100);
  for(int i=0;i<5;i++){
    if(sys_initialized_ == false){
      if(i==4) ROS_ERROR("System isn't inited for a quite long time");
      loop_rate.sleep();
    }
    else
      break;
  }

  double d_orie, d_x, d_y, d_t;
  //计算预测状态
  now_stamp_=ros::Time::now();
  d_t = (now_stamp_-last_stamp_).toSec();
  d_orie = cmdvel_[1]*d_t;  
  d_x = cmdvel_[0]*d_t*cos(pose_t0_[2] + d_orie*0.5);
  d_y = cmdvel_[0]*d_t*sin(pose_t0_[2] + d_orie*0.5);
    //均值
  pose_t1_[0] = pose_t0_[0] + d_x;             //x位置
  pose_t1_[1] = pose_t0_[1] + d_y;             //y位置
  pose_t1_[2] = pose_t0_[2] + d_orie;          //偏航角

    //协方差
  Gt_ <<1,0,(-d_y), 0,1,d_x, 0,0,1;
  cov_t1_ =  Gt_*cov_t0_*(Gt_.inverse()) + Rt_;

  if((odom_actived_ | imu_actived_) != true)
      ROS_ERROR_STREAM("There is no sensor working QAQ");

  //调用局部测量更新函数
  if(imu_actived_ == true){
    for(int i=0;i<5;i++){
      if(imu_ekf_->have_data_==true){
        imu_ekf_ -> update(pose_t1_,cov_t1_); break;
      } 
      else{
        if(i==4) ROS_ERROR("Imu doesn't have data");
        loop_rate.sleep();
      }}}
  if(odom_actived_ == true){
    for(int i=0;i<5;i++){
      if(odom_ekf_->have_data_==true){
        odom_ekf_ -> update(pose_t1_,cov_t1_); break;
      } 
      else{
        if(i==4) ROS_ERROR("Odom doesn't have data");
        loop_rate.sleep();
      }}}

  //判断时间差是否太大
    double diff = fabs((imu_ekf_->now_stamp_ - odom_ekf_->now_stamp_).toSec());
    if (diff > 0.05)
      ROS_ERROR("The time between odom and imu is to long: %f s", diff);

  //计算权重
    //先判断odom数据是否异常,这里也先省略了

    //计算权重
  Matrix3f Mat3_1 = Gt_ * Pij_ * Gt_.transpose() + Rt_;
  Pij_ = (Identity_matrix_ - imu_ekf_->k_) * Mat3_1 * ((Identity_matrix_ - odom_ekf_->k_).transpose());

  P66_.block<3,3>(0,0) << imu_ekf_->last_meas_cov_;
  P66_.block<3,3>(0,3) << Pij_;
  P66_.block<3,3>(3,0) << Pij_;
  P66_.block<3,3>(3,3) << odom_ekf_->last_meas_cov_;
  P66_inverse =  P66_.inverse();

  cov_t0_ = (E63_.transpose()* P66_inverse * E63_).inverse(); //融合之后的协方差矩阵
  A63_ = P66_inverse * E63_ * cov_t0_;
  weight_imu_ << (A63_.transpose()).block<3,3>(0,0);
  weight_odom_ << (A63_.transpose()).block<3,3>(0,3);

  //计算融合结果,并发布
  pose_t0_ = weight_imu_ * imu_ekf_->last_meas_pose_ + weight_odom_ * odom_ekf_->last_meas_pose_;

  odom_tf_.header.stamp = fusion_pub_.header.stamp = now_stamp_;
  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_footprint";
  fusion_pub_.child_frame_id = "odom";
  
  fusion_pub_.pose.pose.position.x= pose_t0_[0];
  fusion_pub_.pose.pose.position.y= pose_t0_[1];
  fusion_pub_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_t0_[2]);
  double tem_covariance[36] = {cov_t0_(0,0),cov_t0_(0,1),  0,  0,  0,cov_t0_(0,2),
                              cov_t0_(1,0),cov_t0_(1,1),   0,  0,  0,cov_t0_(1,2),
                              0,  0,  99999,  0,  0,  0,
                              0,  0,  0,  99999,  0,  0,
                              0,  0,  0,  0,  99999,  0,
                              cov_t0_(2,0),cov_t0_(2,1),  0,  0,  0,cov_t0_(2,2)};
  for(int i = 0; i < 36; i++){
      fusion_pub_.pose.covariance[i] = tem_covariance[i];
  }
  pose_pub_.publish(fusion_pub_);     //发布odom
  odom_tf_.transform.translation.x = fusion_pub_.pose.pose.position.x;
  odom_tf_.transform.translation.y = fusion_pub_.pose.pose.position.y;
  odom_tf_.transform.translation.z = fusion_pub_.pose.pose.position.z;
  odom_tf_.transform.rotation = fusion_pub_.pose.pose.orientation;
  odom_bc_.sendTransform(odom_tf_);  //发布tf

  //更新几个值
  last_stamp_=now_stamp_;
  cmdvel_[0]=cmd_input.linear.x;
  cmdvel_[1]=cmd_input.angular.z;
};

};//end of the namepace FUSION