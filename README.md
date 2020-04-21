--------------------------------------------------
#用于AGV激光雷达导航与SLAM的ROS程序包  
#开发环境为Ubuntu16.04 + ROS Kinetic  
--------------------------------------------------

##使用方法
* __下载__  
    建立一个含src子文件夹的文件夹  
    ` $ mkdir 自定义路径/自定义工作空间名/src`  
    将src设为工作路径  
    ` $ cd 自定义路径/自定义工作空间名/src`  
    拉取代码  
    ` $ git clone git@github.com:OceanYv/gp_agv.git`  
* __编译__  
    编译前，确认是否安装官方的serial功能包。若未安装，运行  
    ` $ sudo apt-get install ros-kinetic-serial`  
    将路径修改为工作空间路径  
    ` $ cd 自定义路径/自定义工作空间名`  
    并运行catkin_make指令以编译程序  
    ` $ catkin_make`  
    运行以下指令将路径刷新到环境变量中  
    ` $ echo "source 自定义路径/自定义工作空间名/devel/setup.bash" >> ~/.bashrc`  
    然后重新打开终端
* __运行__  
    通过rosrun指令或roslaunch指令运行相关程序；


##package说明
* __run_agv：系统初始化，并启动各个模块__  
    启动整个系统的指令为  
    ` $ roslaunch run_agv run_agv.launch`  

* __base_controller：用于基本的运动控制__  
    单独运行用以下指令  
    ` $ roslaunch base_controller base_controller.launch`

* __lslidar_n301：镭神官方提供的ROS开发包，用于读取N301激光雷达数据__  
    单独运行用以下指令  
    ` $ roslaunch lslidar_n301_decoder lslidar_n301.launch --screen`  
    之后运行  
    ` $ rosrun rviz rviz`  
    即可在rviz中即可查看对应激光雷达数据  

* __set_gmapping：使用了gmapping功能包，对一些参数进行设置__  
    可以在set_mapping/launch/robot_gmapping.launch中对参数进行设定  