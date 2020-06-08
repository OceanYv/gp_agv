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
#define PI 3.141593
double x_lim,y_lim,r_lim;

//串口数据收发变量
int odomreq_count;		//当超过ODOMREQ_COUNT时，即说明多次从下位机得到的数据都有问题，产生报错
string rec_buffer;
unsigned char send_buffer[11]={0};
serial::Serial sp;              //Serial对象

//实现char数组和short之间的转换
#define ByteCast(x) ((char)(x))
short char_to_short( const char *b )      //调用的是这个函数
{
    unsigned short ret;
    ret  = (unsigned short)(ByteCast(b[1]));
    ret |= (unsigned short)(ByteCast(b[0])) << 8;
    return (short)ret;
}

//回调函数，发送线速度、角速度
void Callback(const geometry_msgs::Twist &cmd_input){
    double x,y,r;
    x = cmd_input.linear.x;
    y = cmd_input.linear.y;
    r = cmd_input.angular.z;
    ROS_INFO("x = %lf",x);
    if(abs(x) > x_lim)
        x = x > 0 ? x_lim : -x_lim;
    if(abs(y) > y_lim)
        y = y > 0 ? y_lim : -y_lim;
    if(abs(r) > r_lim)
        r = r > 0 ? r_lim : -r_lim;

    char velx, vely,velr;
    velx = (char)(x * 100);         //m转化为cm
    vely = (char)(y * 100);
    velr = (char)(r * 180 / PI) / 2.0;  //rad转化为°

    //新协议：配置速度控制指令串口命令
    send_buffer[0] = 0xFE;
    send_buffer[1] = 0xEE;
    send_buffer[2] = 0x04;              //长度
    send_buffer[3] = CMD_SPEEDSET;      //命令类型
    send_buffer[4] = velx;      //x方向线速度
    send_buffer[5] = vely;      //y方向线速度
    send_buffer[6] = velr;      //角速度
    send_buffer[7] = 0x01;
    send_buffer[8] = 0x01;
    send_buffer[9] = 0xFC;
    send_buffer[10] = 0xFF;

    //老协议：配置速度控制指令串口命令
/*    send_buffer[0] = 0x52;
    send_buffer[1] = 0x54;
    send_buffer[2] = CMD_SPEEDSET;
    send_buffer[3] = 0x04;
    
    send_buffer[4] = 0x01;    //movetype
    send_buffer[5] = velx;    //x方向线速度
    send_buffer[6] = vely;      //y方向线速度
    send_buffer[7] = velr;   //角速度
    send_buffer[8] = send_buffer[4] + send_buffer[5] + send_buffer[6] + send_buffer[7];
    send_buffer[9] = 0x0A;
    send_buffer[10] = 0x0D;*/

    //串口发送
    if(sp.write(send_buffer,11) != 11)
        ROS_ERROR("velcmd sent failed");
    else
        ROS_INFO("velcmd sent succeed");
}

//向下位机请求里程计数据
void Odom_request(void){

    //新协议
    send_buffer[0] = 0xFE;
    send_buffer[1] = 0xEE;
    send_buffer[2] = 0x01;              //长度
    send_buffer[3] = CMD_ODOMREQUEST;      //命令类型
    send_buffer[4] = 0x01;
    send_buffer[5] = 0x01;
    send_buffer[6] = 0x01;
    send_buffer[7] = 0x01;
    send_buffer[8] = 0x01;
    send_buffer[9] = 0xFC;
    send_buffer[10] = 0xFF;

/*    //老协议
    send_buffer[0] = 0x52;
    send_buffer[1] = 0x54;
    send_buffer[2] = CMD_ODOMREQUEST;              //长度
    send_buffer[3] = 0x01;      //命令类型

    send_buffer[4] = 0x01;

    send_buffer[5] = send_buffer[4];
    send_buffer[6] = 0x0A;
    send_buffer[7] = 0x0D;*/


//串口发送
    if(sp.write(send_buffer,11) != 11)
        ROS_ERROR("Request sent failed");
    else
        ROS_INFO("Request sent succeed");
    odomreq_count++;

}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    
//串口通信配置
    string serial_stm;
    int baud_stm,timeout;
    nh.param<std::string>("/serial_congif/SERIAL_STM",serial_stm,"ttyUSB0");
    nh.param("/serial_congif/BAUD_STM",baud_stm,9600);
    nh.param("/serial_congif/TIMEOUT",timeout,1000);

    nh.param("/speed_limit/x",x_lim,1.0);
    nh.param("/speed_limit/y",y_lim,0.1);
    nh.param("/speed_limit/r",r_lim,1.0);

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
    ros::Subscriber sub = nh.subscribe("cmd_vel",2, Callback);
    ros::Publisher odom_pub= nh.advertise<nav_msgs::Odometry>("odom", 2);   //发布里程计信息
    static tf::TransformBroadcaster odom_bc;    //发布tf_tree
//定义变量
    int rate;
    double pos_temp[3],pos_deta[3];
    ros::Time now_stamp,last_stamp;             //时间帧，也用于计算速度位移
    ros::Duration dt_Duration;
    short x_sum_meas,y_sum_meas,r_sum_meas;
    nav_msgs::Odometry odom_inf;                //里程计信息
    geometry_msgs::TransformStamped odom_tf;    //tf转换信息
    geometry_msgs::Point odom_point;            //用于发布里程计位置信息
    geometry_msgs::Vector3 odom_point_tf;       //用于发送tf位移信息
    double orie;                                //航偏角
    geometry_msgs::Quaternion odom_quat;        //姿态信息（四元数）
    geometry_msgs::Vector3 vel_linear,vel_angular;      //线速度、角速度
    float covariance[36] = {0.01,   0,  0,  0,  0,  0,  //位置和速度的测量不确定性协方差矩阵
                            0,  0.01,   0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,  0.01};
    for(int i = 0; i < 36; i++){
        odom_inf.pose.covariance[i] = covariance[i];
    }

//初始化变量
    odomreq_count=0;
    pos_temp[0]=pos_temp[1]=pos_temp[2]=0;
    pos_deta[0]=pos_deta[1]=pos_deta[2]=0;
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
        //while(!sp.waitReadable());
        rec_buffer =sp.read(17);    //新协议
        //rec_buffer =sp.readline(16,"\n");    //旧协议
        const char *receive_data = rec_buffer.data(); //保存串口发送来的数据
        
        if(receive_data[0]==0xFE && receive_data[1]==0xEE && receive_data[15]==0xFC && receive_data[16]==0xFF)    //新
        //if(receive_data[0]==0x52 && receive_data[1]==0x54 && receive_data[14]==0x0A && (receive_data[15]==0x0D || receive_data[15]==0x0A))      //旧
        { //校验
            odomreq_count=0;

            //计算时间间隔并转化为秒
            now_stamp=ros::Time::now();
            dt_Duration=now_stamp-last_stamp;
            last_stamp=now_stamp;
            double dt=dt_Duration.toSec(); 
            
            x_sum_meas = char_to_short(receive_data+4);    //x方向，放大了100倍,mm
            y_sum_meas = char_to_short(receive_data+6);
            r_sum_meas = char_to_short(receive_data+8);    //角位移，放大了100倍，°

            //新协议
            pos_temp[0]=x_sum_meas/100000.0;     //x方向位置,m
            pos_temp[1]=y_sum_meas/100000.0;     //y方向位置,m
            pos_temp[2]=r_sum_meas*PI/18000.0;     //方向,弧度.范围不局限于-pi°～pi°
            //更新绝对速度
            vel_linear.x=((pos_temp[0]-odom_point.x)/dt+vel_linear.x)/2;
            vel_linear.y=((pos_temp[1]-odom_point.y)/dt+vel_linear.y)/2;
            vel_angular.z=((pos_temp[2]-orie)/dt+vel_angular.z)/2;

            //旧协议
           /* pos_deta[0]=x_sum_meas/100000.0;     //△t内小车坐标系下x方向位移,m
            pos_deta[1]=y_sum_meas/100000.0;     //y方向位移,m
            pos_deta[2]=r_sum_meas*PI/180000.0;     //方向,弧度

            //更新绝对速度
            vel_linear.x=(pos_deta[0]/dt+vel_linear.x)/2;
            vel_linear.y=(pos_deta[1]/dt+vel_linear.y)/2;
            vel_angular.z=(pos_deta[2]/dt+vel_angular.z)/2;

            //计算位姿
            pos_temp[2] += pos_deta[2];
            pos_temp[2] = (pos_temp[2] > PI) ? (pos_temp[2] - 2*PI) : ((pos_temp[2] < -PI) ? (pos_temp[2] + 2*PI) : pos_temp[2]);
            pos_temp[0] += cos(pos_temp[2]) * pos_deta[0] - sin(pos_temp[2]) * pos_deta[1];
            pos_temp[1] += sin(pos_temp[2]) * pos_deta[0] + cos(pos_temp[2]) * pos_deta[1];//旧协议到这里*/

            //更新全局位姿态
            odom_point_tf.x=odom_point.x=pos_temp[0];
            odom_point_tf.y=odom_point.y=pos_temp[1];
            orie=pos_temp[2];
            odom_quat = tf::createQuaternionMsgFromYaw(orie);   //偏航角转换成四元数

          //现在要用自己写的数据融合程序来发布这个tf变换，所以把这几行注释掉了  
            //发布tf坐标变化
            odom_tf.header.stamp = now_stamp; 
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base_footprint";
            odom_tf.transform.translation = odom_point_tf;
            odom_tf.transform.rotation = odom_quat;        
            odom_bc.sendTransform(odom_tf);
            //发布里程计信息
            odom_inf.header.stamp = now_stamp; 
            odom_inf.header.frame_id = "odom";  //位置是在odom坐标系下的
            odom_inf.child_frame_id = "base_footprint"; //速度是在odom坐标系下的
            odom_inf.pose.pose.position= odom_point;
            odom_inf.pose.pose.orientation = odom_quat;       
            odom_inf.twist.twist.linear = vel_linear;
            odom_inf.twist.twist.angular = vel_angular;
            odom_pub.publish(odom_inf);
        }
/*        
        else{
                ROS_ERROR_STREAM("Fail to request odom data from STM32.");
        }*/
        ros::spinOnce();    //每次循环调用一次回调函数，向下位机发送指定速度
        loop_rate.sleep();
    }
    return 0;
}
