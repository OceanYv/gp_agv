/*
 *在机器人启动阶段调用
 *发布map-odom、base-laser等固定的tf信息
 *需要调用参数服务器中的数据
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>	//用于tf

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_bulid");
    ros::NodeHandle nh_run;

//定义变量
    static tf::TransformBroadcaster tf_bc;    //发布tf信息

    ros::Time now_stamp;             //时间帧
    geometry_msgs::TransformStamped info_tf;    //tf转换信息
    double orie;                                //航偏角
    geometry_msgs::Vector3 info_point_tf;       //用于发送tf位移信息
    geometry_msgs::Quaternion info_quat;        //姿态信息（四元数）

    ros::Rate loop_rate(1);

//map-odom,只发布一次，之后由gmapping更新数据
    info_point_tf.z=0;
    now_stamp=ros::Time::now();

    nh_run.param("fixed_tf/map_odom_tf/x",info_point_tf.x,0.0);
    nh_run.param("fixed_tf/map_odom_tf/y",info_point_tf.y,0.0);
    nh_run.param("fixed_tf/map_odom_tf/orie",orie,0.0);
    info_quat = tf::createQuaternionMsgFromYaw(orie);   ////偏航角转换成四元数
    //发布tf坐标变化
    info_tf.header.stamp = now_stamp;
    info_tf.header.frame_id = "map";
    info_tf.child_frame_id = "odom";
    info_tf.transform.translation = info_point_tf;
    info_tf.transform.rotation = info_quat;        
    tf_bc.sendTransform(info_tf);

    while(ros::ok()){
        info_point_tf.z=0;
        now_stamp=ros::Time::now();

    //base-laser
        nh_run.param("fixed_tf/base_laser_tf/x",info_point_tf.x,0.0);
        nh_run.param("fixed_tf/base_laser_tf/y",info_point_tf.y,0.0);
        nh_run.param("fixed_tf/base_laser_tf/orie",orie,0.0);
        info_quat = tf::createQuaternionMsgFromYaw(orie);   ////偏航角转换成四元数

        //发布tf坐标变化
        info_tf.header.frame_id = "base_footprint";
        info_tf.child_frame_id = "laser";
        info_tf.transform.translation = info_point_tf;
        info_tf.transform.rotation = info_quat;        
        tf_bc.sendTransform(info_tf);  

        loop_rate.sleep();
    }
    return 0;
}