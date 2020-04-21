******************************************************
*********用于AGV激光雷达导航与SLAM的ROS程序包*********
*********开发环境为Ubuntu16.04 + ROS Kinetic**********
******************************************************

#使用方法
1.下载
    下载后，将gp_agv文件夹放在workspace下的src文件夹中；
2.编译
    编译前，确认是否安装官方的serial功能包。若未安装，运行<sudo apt-get install ros-kinetic-serial>指令；
    在工作空间路径下运行<catkin_make>指令，进行编译；
3.运行
    通过rosrun指令或roslaunch指令运行相关程序；


#package说明
1.base_controller：用于基本的运动控制
    单独运行：<roslaunch base_controller base_controller.launch>

2.set_gmapping：使用了gmapping功能包，对一些参数进行设置

3.lslidar_n301：镭神官方提供的ROS开发包，用于读取N301激光雷达数据
    单独运行：<roslaunch lslidar_n301_decoder lslidar_n301.launch --screen>
之后<rosrun rviz rviz>，在rviz中即可查看对应激光雷达数据
