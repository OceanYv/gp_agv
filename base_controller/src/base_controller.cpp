#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>		//用于接收CMD_VEL节点
#include <tf/transform_broadcaster.h>	//用于tf
#include <nav_msgs/Odometry.h>			//用于里程计

#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <serial/serial.h>

//通信协议部分参数
#define CMD_SPEEDSET 'm'        //速度设置指令


using std::string;

//串口数据收发变量
string rec_buffer;
unsigned char send_buffer[10]={0};
serial::Serial sp;              //Serial对象

//实现char数组和float之间的转换
union floatData {
    float d;
    unsigned char data[4];
}vel_linear_set,vel_angular_set,vel_linear_meas,vel_angular_meas;
//设定线速度      设定角速度       测定线速度        测定角速度,单位m，s

//回调函数，获取线速度、角速度，单位为m，s
void Callback(const geometry_msgs::Twist &cmd_input){
    
    //获取速度
    vel_linear_set.d = cmd_input.linear.x;
    vel_angular_set.d = cmd_input.angular.z;
    //配置串口命令
    send_buffer[0] = CMD_SPEEDSET;
    for(int i=0;i<4;i++){    
        send_buffer[i+1]=vel_linear_set.data[i];
        send_buffer[i+5]=vel_angular_set.data[i];
    }
    send_buffer[9] = 0x0d;      // \n作为结束符
    //串口发送
    sp.write(send_buffer,10);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
//串口通信配置
    string serial_stm;
    int baud_stm,timeout,rate;
    nh.param<std::string>("/serial_congif/SERIAL_STM",serial_stm,"ttyUSB0");
    nh.param("/serial_congif/BAUD_STM",baud_stm,115200);
    nh.param("/serial_congif/TIMEOUT",timeout,1000);
    nh.param("/serial_congif/RATE",rate,10);

    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);   //串口通信超时时间
    sp.setPort(serial_stm);
    sp.setBaudrate(baud_stm);
    sp.setTimeout(to);
//打开串口
    try{
        sp.open();
    }
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    if(sp.isOpen())
        ROS_INFO_STREAM("Serial is opened.");
    else
        return -1;
//定义发布、接收
    ros::Subscriber sub = nh.subscribe("cmd_vel",10, Callback);
    ros::Publisher odom_pub= nh.advertise<nav_msgs::Odometry>("odom", 10);   //发布里程计信息
    static tf::TransformBroadcaster odom_bc;    //发布tf_tree
//定义变量
    ros::Time now_stamp,last_stamp;             //时间帧，也用于计算速度位移
    ros::Duration dt_Duration;
    nav_msgs::Odometry odom_inf;                //里程计信息
    geometry_msgs::TransformStamped odom_tf;    //tf转换信息
    geometry_msgs::Point odom_point;            //用于发布里程计位置信息
    geometry_msgs::Vector3 odom_point_tf;       //用于发送tf位移信息
    double orie;                                //航偏角
    geometry_msgs::Quaternion odom_quat;        //姿态信息（四元数）
    geometry_msgs::Vector3 vel_linear,vel_angular;    //线速度、角速度
//  float covariance[36] = {0.01,   0,  0,  0,  0,  0,  //位置和速度的测量不确定性协方差矩阵
//                          0,  0.01,   0,  0,  0,  0,
//                          0,  0,  99999,  0,  0,  0,
//                          0,  0,  0,  99999,  0,  0,
//                          0,  0,  0,  0,  99999,  0,
//                          0,  0,  0,  0,  0,  0.01};
//  for(int i = 0; i < 36; i++)
//      odom_tf.pose.covariance[i] = covariance[i];
//      odom_inf.pose.covariance[i] = covariance[i];

//初始化变量
    odom_point.x=odom_point.y=odom_point.z=0;
    odom_point_tf.x=odom_point_tf.y=odom_point_tf.z=0;
    odom_quat.x=odom_quat.y=odom_quat.z=odom_quat.w=0;
    orie=0;
    vel_linear.x=vel_linear.y=vel_linear.z=0;
    vel_angular.x=vel_angular.y=vel_angular.z=0;
    last_stamp=now_stamp=ros::Time::now();

    ros::Rate loop_rate(rate);

//处理串口信息，计算并发布tf转换、odom_inf
    while(ros::ok()){        

//这边应该加一个向STM32请求里程计数据的指令，然后再接收数据，这样计时会更加准确
//这边应该加一个向STM32请求里程计数据的指令，然后再接收数据，这样计时会更加准确
//这边应该加一个向STM32请求里程计数据的指令，然后再接收数据，这样计时会更加准确

    //计算时间间隔并转化为秒
        double dt=dt_Duration.toSec(); 
        now_stamp=ros::Time::now();
        dt_Duration=now_stamp-last_stamp;
        last_stamp=now_stamp;
    //读取串口数据，格式：‘指令符’+测定线速度+测定角速度+‘终止符’（10字节）,单位m，s
        rec_buffer =sp.readline(10,"\n");    //获取串口发送来的数据
        const char *receive_data=rec_buffer.data(); //保存串口发送来的数据
        for(int i=0;i<4;i++){
            vel_linear_meas.data[i]=receive_data[i+1];
            vel_angular_meas.data[i]=receive_data[i+5];
        }
    //相对速度
        double d_linear_meas=vel_linear_meas.d*dt;      //相对位移
        double d_angular_meas=vel_angular_meas.d*dt;    //相对转角
    //更新绝对速度
        vel_linear.x=vel_linear_meas.d*cos(orie);
        vel_linear.y=vel_linear_meas.d*sin(orie);
        vel_angular.z=vel_angular_meas.d;
    //更新全局位姿
        if (d_linear_meas != 0){
            double dx = cos(d_angular_meas) * d_linear_meas;
            double dy = -sin(d_angular_meas) * d_linear_meas;
            odom_point.x += (cos(orie) * dx - sin(orie) * dy);
            odom_point.y += (sin(orie) * dx + cos(orie) * dy);
        }
        if (vel_angular_meas.d != 0)
            orie += vel_angular_meas.d;
        odom_point_tf.y=odom_point.y;
        odom_point_tf.y=odom_point.y;
		odom_quat = tf::createQuaternionMsgFromYaw(orie);   ////偏航角转换成四元数

    //发布tf坐标变化
        odom_tf.header.stamp = now_stamp;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";
        odom_tf.transform.translation = odom_point_tf;
        odom_tf.transform.rotation = odom_quat;        
        odom_bc.sendTransform(odom_tf);    
        
    //发布里程计信息
        odom_inf.header.stamp = now_stamp; 
        odom_inf.header.frame_id = "odom";
        odom_inf.child_frame_id = "base_footprint";
        odom_inf.pose.pose.position= odom_point;
        odom_inf.pose.pose.orientation = odom_quat;       
        odom_inf.twist.twist.linear = vel_linear;
        odom_inf.twist.twist.angular = vel_angular;
        odom_pub.publish(odom_inf);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
