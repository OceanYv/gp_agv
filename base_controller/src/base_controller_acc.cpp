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
#define PI 3.141593
#define SMO_K 0.2                 //运动平滑控制系数
#define SMO_LIM_V 0.05            //速度平滑减速阈值(m/s)
#define SMO_LIM_W 0.1            //角速度平滑减速阈值(rad/s)
double x_lim,y_lim,r_lim;

//串口数据收发变量
string rec_buffer;
unsigned char send_buffer[11]={0};
serial::Serial sp;              //Serial对象
uint8_t receive_data[20];

//实现char数组和short之间的转换
short uint8_to_short( uint8_t *b )      //调用的是这个函数
{
    unsigned short ret;

    ret  = (unsigned short)(b[1]);
    ret |= (unsigned short)(b[0]) << 8;

    return (short)ret;
}

//回调函数，发送线速度、角速度
double x = 0, y = 0, r = 0;
void Callback(const geometry_msgs::Twist &cmd_input){
    //获取vel数据
    double x_r,y_r,r_r;
    x_r = cmd_input.linear.x;
    y_r = cmd_input.linear.y;
    r_r = cmd_input.angular.z;
    if(fabs(x_r) > x_lim)
        x_r = x_r > 0 ? x_lim : -x_lim;
    if(fabs(y) > y_lim)
        y_r = y_r > 0 ? y_lim : -y_lim;
    if(fabs(r_r) > r_lim)
        r_r = r_r > 0 ? r_lim : -r_lim;

    //速度平滑化
    x = SMO_K * (x_r - x) + x;
    if((fabs(x) < SMO_LIM_V) && ((fabs(x_r) - fabs(x))<0)) x = 0;
    y = SMO_K * (y_r - y) + y;
    if((fabs(y) < SMO_LIM_V) && ((fabs(y_r) - fabs(y))<0)) y = 0;
    r = SMO_K * (r_r - r) + r;
    if((fabs(r) < SMO_LIM_W) && ((fabs(r_r) - fabs(r))<0)) r = 0;

    //单位转换、数组填充与发送
    char velx, vely, velr;
    velx = (char)(x * 100);         //m转化为cm
    vely = (char)(y * 100);
    velr = (char)(r * 180 / PI) / 2.0;  //rad转化为°

    send_buffer[0] = 0xFE;
    send_buffer[1] = 0xEE;
    send_buffer[2] = 0x05;              //长度
    send_buffer[3] = CMD_SPEEDSET;      //命令类型
    send_buffer[4] = 0x01;
    send_buffer[5] = velx;      //x方向线速度
    send_buffer[6] = vely;      //y方向线速度
    send_buffer[7] = velr;      //角速度
    send_buffer[8] = 0x55;
    send_buffer[9] = 0xaa;
    send_buffer[10] = 0xFC;
    send_buffer[11] = 0xFF;

    //串口发送
    if(sp.write(send_buffer,12) != 12)
        ROS_ERROR("velcmd sent failed");
}

//向下位机请求里程计数据
void Odom_request(void){

    send_buffer[0] = 0xFE;
    send_buffer[1] = 0xEE;
    send_buffer[2] = 0x05;              //长度
    send_buffer[3] = CMD_ODOMREQUEST;      //命令类型
    send_buffer[4] = 0x01;
    send_buffer[5] = 0x01;
    send_buffer[6] = 0x01;
    send_buffer[7] = 0x01;
    send_buffer[8] = 0x55;
    send_buffer[9] = 0xaa;
    send_buffer[10] = 0xFC;
    send_buffer[11] = 0xFF;

//串口发送
    if(sp.write(send_buffer,12) != 12)
        ROS_ERROR("Request sent failed");
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "base_controller_acc");
    ros::NodeHandle nh;
    
//串口通信配置
    string serial_stm;
    int baud_stm,timeout;
    nh.param<std::string>("/serial_congif/SERIAL_STM",serial_stm,"ttyUSB0");
    nh.param("/serial_congif/BAUD_STM",baud_stm,9600);
    nh.param("/serial_congif/TIMEOUT",timeout,1000);

    nh.param("/speed_limit/x",x_lim,1.5);
    nh.param("/speed_limit/y",y_lim,0.0);
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
        ROS_INFO("Serial is opened.");
    else 
        return -1;
//定义发布、接收
    ros::Subscriber sub = nh.subscribe("cmd_vel",2, Callback);
    ros::Publisher odom_pub= nh.advertise<nav_msgs::Odometry>("odom", 2);   //发布里程计信息
    static tf::TransformBroadcaster odom_bc;    //发布tf_tree
//定义变量
    int rate;
    double pos_temp[3];
    ros::Time now_stamp,last_stamp;             //时间帧，也用于计算速度位移
    ros::Duration dt_Duration;
    double dx,dy,dz;
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
        while(!sp.waitReadable());
        sp.read(receive_data,17);

        if(receive_data[0]==0xFE && receive_data[1]==0xEE && receive_data[15]==0xFC && receive_data[16]==0xFF)    //新
        { //校验
            //计算时间间隔并转化为秒
            now_stamp=ros::Time::now();
            dt_Duration=now_stamp-last_stamp;
            last_stamp=now_stamp;
            double dt=dt_Duration.toSec(); 
            
            dx = 1.0*uint8_to_short(receive_data+4)/100000.0;    //单位时间内在机器人坐标系下x方向位移，放大了100倍,mm;所以这里处理之后转化为m
            dy = -1.0*uint8_to_short(receive_data+6)/100000.0;
            dz = 1.0*uint8_to_short(receive_data+8)*PI/180000.0;    //角位移，放大了1000倍，°，所以这里处理之后转化为弧度

            //计算机器人坐标系下的速度
            vel_linear.x =( dx / dt + vel_linear.x) / 2.0;
            vel_linear.y =( dy / dt + vel_linear.y) / 2.0;
            vel_angular.z =( dz / dt + vel_angular.z) / 2.0;

            //更新全局位姿态
            orie += dz;
            orie = (orie > PI) ? (orie - 2*PI) : ((orie < -PI) ? (orie + 2*PI) : orie);
            
            pos_temp[0] += cos(orie) * dx - sin(orie) * dy;
            pos_temp[1] += sin(orie) * dx + cos(orie) * dy;
            //ROS_INFO("%f %f %f",pos_temp[0],pos_temp[1],orie);
            odom_point_tf.x=odom_point.x=pos_temp[0];
            odom_point_tf.y=odom_point.y=pos_temp[1];
            odom_quat = tf::createQuaternionMsgFromYaw(orie);   //偏航角转换成四元数

          //现在要用自己写的数据融合程序来发布这个tf变换，所以把这几行注释掉了  
            //发布tf坐标变化
            odom_tf.header.stamp = ros::Time::now(); 
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base_footprint";
            odom_tf.transform.translation = odom_point_tf;
            odom_tf.transform.rotation = odom_quat;        
            odom_bc.sendTransform(odom_tf);
            //发布里程计信息
            odom_inf.header.stamp = odom_tf.header.stamp; 
            odom_inf.header.frame_id = "odom";  //位置是在odom坐标系下的
            odom_inf.child_frame_id = "base_footprint"; //速度是在odom坐标系下的
            odom_inf.pose.pose.position= odom_point;
            odom_inf.pose.pose.orientation = odom_quat;       
            odom_inf.twist.twist.linear = vel_linear;
            odom_inf.twist.twist.angular = vel_angular;
            odom_pub.publish(odom_inf);
        }
        else{
            ROS_ERROR("Fail to get right odom data from STM32.");
        }
        
        ros::spinOnce();    //每次循环调用一次回调函数，向下位机发送指定速度
        loop_rate.sleep();
    }
    return 0;
}
