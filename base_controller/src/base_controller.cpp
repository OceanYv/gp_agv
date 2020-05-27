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
using std::string;

//通信协议部分参数
#define CMD_SPEEDSET 0x01        //速度控制指令
#define CMD_ODOMREQUEST 0x02        //请求下位机数据指令
#define ODOMREQ_COUNT 5            //反复请求里程计数据最大次数

//串口数据收发变量
int odomreq_count;		//当超过ODOMREQ_COUNT时，即说明多次从下位机得到的数据都有问题，产生报错
string rec_buffer;
const char *receive_data;
unsigned char send_buffer[11]={0};
serial::Serial sp;              //Serial对象

//实现char数组和short之间的转换
union shortData {
    int16_t d;
    unsigned char data[2];
}x_sum_meas,y_sum_meas,r_sum_meas;
//测定线速度        测定角速度

//回调函数，发送线速度、角速度
void Callback(const geometry_msgs::Twist &cmd_input){
    char vel_linear_set,vel_angular_set;
    //获取速度
    vel_linear_set = (char)(cmd_input.linear.x/10);             //单位mm/s转化为cm/s
    vel_angular_set = (char)(cmd_input.angular.z*57.29578);     //单位rad/s转化为°/s
    //配置速度控制指令串口命令
    send_buffer[0] = 0xFE;
    send_buffer[1] = 0xEE;
    send_buffer[2] = 0x04;              //长度
    send_buffer[3] = CMD_SPEEDSET;      //命令类型
    send_buffer[4] = vel_linear_set;    //x方向线速度
    send_buffer[5] = 0x00;              //y方向线速度
    send_buffer[6] = vel_angular_set;   //角速度
    send_buffer[9] = 0xFC;
    send_buffer[10] = 0xFF;
    //串口发送
    sp.write(send_buffer,11);
}

//向下位机请求里程计数据
void Odom_request(void){
    send_buffer[0] = 0xFE;
    send_buffer[1] = 0xEE;
    send_buffer[2] = 0x01;              //长度
    send_buffer[3] = CMD_ODOMREQUEST;      //命令类型
    send_buffer[6] = 0xFC;
    send_buffer[7] = 0xFF;
//串口发送
    sp.write(send_buffer,8);
    odomreq_count++;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    
//串口通信配置
    string serial_stm;
    int baud_stm,timeout;
    nh.param<std::string>("/serial_congif/SERIAL_STM",serial_stm,"ttyUSB0");
    nh.param("/serial_congif/BAUD_STM",baud_stm,115200);
    nh.param("/serial_congif/TIMEOUT",timeout,1000);

    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);   //串口通信超时时间
    sp.setPort(serial_stm);
    sp.setBaudrate(baud_stm);
    sp.setTimeout(to);
//打开串口
    try{sp.open();}
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    if(sp.isOpen())
        ROS_WARN_STREAM("Serial is opened.");
    else 
        return -1;
//定义发布、接收
    ros::Subscriber sub = nh.subscribe("cmd_vel",10, Callback);
    ros::Publisher odom_pub= nh.advertise<nav_msgs::Odometry>("odom_data", 10);   //发布里程计信息
    static tf::TransformBroadcaster odom_bc;    //发布tf_tree
//定义变量
    int rate;
    double pos_temp[3];
    ros::Time now_stamp,last_stamp;             //时间帧，也用于计算速度位移
    ros::Duration dt_Duration;
    nav_msgs::Odometry odom_inf;                //里程计信息
    geometry_msgs::TransformStamped odom_tf;    //tf转换信息
    geometry_msgs::Point odom_point;            //用于发布里程计位置信息
    geometry_msgs::Vector3 odom_point_tf;       //用于发送tf位移信息
    double orie;                                //航偏角
    geometry_msgs::Quaternion odom_quat;        //姿态信息（四元数）
    geometry_msgs::Vector3 vel_linear,vel_angular;    //线速度、角速度
    float covariance[36] = {0.01,   0,  0,  0,  0,  0,  //位置和速度的测量不确定性协方差矩阵
                            0,  0.01,   0,  0,  0,  0,
                            0,  0,  99999,  0,  0,  0,
                            0,  0,  0,  99999,  0,  0,
                            0,  0,  0,  0,  99999,  0,
                            0,  0,  0,  0,  0,  0.01};
    for(int i = 0; i < 36; i++){
        odom_inf.pose.covariance[i] = covariance[i];
    }

//初始化变量
    odomreq_count=0;
    pos_temp[0]=pos_temp[1]=pos_temp[2]=0;
    nh.param("/serial_congif/RATE",rate,10);
    odom_point.x=odom_point.y=odom_point.z=0;
    odom_point_tf.x=odom_point_tf.y=odom_point_tf.z=0;
    orie=0;
    odom_quat = tf::createQuaternionMsgFromYaw(orie);   ////偏航角转换成四元数*/
    vel_linear.x=vel_linear.y=vel_linear.z=0;
    vel_angular.x=vel_angular.y=vel_angular.z=0;
    last_stamp=now_stamp=ros::Time::now();

    ros::Rate loop_rate(rate);
    while(ros::ok()){           //处理串口信息，计算并发布tf转换、odom_inf
        //请求下位机数据，并读取串口数据
        Odom_request();
        rec_buffer =sp.readline(17,"\n");    //获取串口发送来的数据
        receive_data=rec_buffer.data(); //保存串口发送来的数据

        if(receive_data[0]==0xFE && receive_data[1]==0xEE && receive_data[15]==0xFC && receive_data[16]==0xFF){ //校验
            odomreq_count=0;
            if(receive_data[3]==0x00){      //编码器里程反馈
            //计算时间间隔并转化为秒
                now_stamp=ros::Time::now();
                dt_Duration=now_stamp-last_stamp;
                last_stamp=now_stamp;
                double dt=dt_Duration.toSec(); 

                for(int i=0;i<2;i++){
                    x_sum_meas.data[i]=receive_data[i+4];
                    y_sum_meas.data[i]=receive_data[i+6];
                    r_sum_meas.data[i]=receive_data[i+8];
                }
                pos_temp[0]=((double)x_sum_meas.d)/100;     //x方向位置,mm
                pos_temp[1]=((double)y_sum_meas.d)/100;     //y方向位置,mm
                pos_temp[2]=((double)r_sum_meas.d)/5729.578;     //方向,弧度
                //更新绝对速度
                vel_linear.x=(pos_temp[0]-odom_point.x)/dt;
                vel_linear.y=(pos_temp[1]-odom_point.y)/dt;
                vel_angular.z=(pos_temp[2]-orie)/dt;
                //更新全局位姿态
                odom_point_tf.x=odom_point.x=pos_temp[0];
                odom_point_tf.y=odom_point.y=pos_temp[1];
                orie=pos_temp[2];
                odom_quat = tf::createQuaternionMsgFromYaw(orie);   //偏航角转换成四元数
/*          现在要用自己写的数据融合程序来发布这个tf变换，所以把这几行注释掉了  
            //发布tf坐标变化
                odom_tf.header.stamp = now_stamp; 
                odom_tf.header.frame_id = "odom";
                odom_tf.child_frame_id = "base_footprint";
                odom_tf.transform.translation = odom_point_tf;
                odom_tf.transform.rotation = odom_quat;        
                odom_bc.sendTransform(odom_tf);   */ 
            //发布里程计信息
                odom_inf.header.stamp = now_stamp; 
                odom_inf.header.frame_id = "odom";  //位置是在odom坐标系下的
                odom_inf.child_frame_id = "odom"; //速度是在odom坐标系下的
                odom_inf.pose.pose.position= odom_point;
                odom_inf.pose.pose.orientation = odom_quat;       
                odom_inf.twist.twist.linear = vel_linear;
                odom_inf.twist.twist.angular = vel_angular;
                odom_pub.publish(odom_inf);
            }       
        }
        else{
            if(odomreq_count>=ODOMREQ_COUNT)
                ROS_ERROR_STREAM("Fail to request odom data from STM32.");
            else
                Odom_request();
        }
        ros::spinOnce();    //每次循环调用一次回调函数，向下位机发送指定速度
        loop_rate.sleep();
    }
    return 0;
}
