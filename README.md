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
    编译前，确认是否安装官方的serial功能包、robot_pose_ekf功能包，确认方法为在终端中输入  
        ` $ rospack find 包名`  
        若未安装serial，运行  
        ` $ sudo apt-get install ros-kinetic-serial`  
        若未安装robot_pose_ekf，在 https://github.com/ros-planning/navigation/tree/kinetic-devel 下载并编译；  
    将路径修改为工作空间路径  
        ` $ cd 自定义路径/自定义工作空间名`  
    并运行catkin_make指令以编译程序  
        ` $ catkin_make`  
    运行以下指令将路径刷新到环境变量中  
        ` $ echo "source 自定义路径/自定义工作空间名/devel/setup.bash" >> ~/.bashrc`  
    然后重新打开终端即可使用
* __运行__  
    通过rosrun指令或roslaunch指令运行相关程序；


##package说明
* __run_agv：系统初始化，并启动各个模块__  
    启动整个系统的指令为  
    ` $ roslaunch run_agv run_agv.launch`  
    __要设置的参数__  
	/config/fixed_tf.yaml中的参数:激光雷达安装位置信息  

* __base_controller：用于基本的运动控制、里程计数据和IMU数据获取与融合__
    单独运行用以下指令  
    ` $ roslaunch base_controller base_controller.launch`  
    __要设置的参数__  
	/config/base_controller.yaml中的参数:STM32串口通讯参数  
	/launch/include/kenzhrobot_pose_ekf.launch.xml中的参数：有注释的三行  
    __文件说明__  
    base_controller.cpp:获取速度指令后通过串口向下位机发送速度指令，并并定时读取下位机的里程计数据；  

* __lslidar_n301：镭神官方提供的ROS开发包，用于读取N301激光雷达数据__  
    单独运行用以下指令  
    ` $ roslaunch lslidar_n301_decoder lslidar_n301.launch --screen`  
    之后运行  
    ` $ rosrun rviz rviz`  
    即可在rviz中即可查看对应激光雷达数据  

* __set_gmapping：调用gmapping功能包,并保存地图文件__  
    __要设置的参数__  
	/launch/robot_gmapping.launch中的参数:gma pping参数、save_map参数  
    该launch文件中的file_location和mapname两个param一定要按照自己的位置来配置  

* __hwtimu:imu提供的包，用于通过串口连接imu并发布imu数据topic__  
    单独运行用以下指令  
    ` $ roslaunch hwtimu hwtimusubexp.launch`  
    __要设置的参数__  
	/cfg下yaml文件中的参数  

* __joy_control、:遥控器使用和驱动，通过无线通讯控制底盘的运动，发布vel_cmd话题__
    单独运行用以下指令  
    ` $ roslaunch joy_control we_joy_control.launch`  

##其他说明
*__硬件要求__  
    在安装①网口通讯的激光雷达②RS232通讯的STM32 后方可正常运行；  
    若在无上诉硬件时运行run_agv.launch，会报错；  
    如果想要观察系统结构，请将/base_controller/src/base_controller.cpp中的71-91、133-135\182-187行注释掉，以免base_controller节点被kill；  

*__一些基本指令__  
  查看tf tree  
  ` $  rosrun rqt_tf_tree rqt_tf_tree`  
  查看系统结构
  ` $ rqt_graph `  
  查看参数服务器  
  ` $ rosparam list `  


  2020.6.1程序调试  
    roscore  
    rqt_graph  
    rosrun rqt_tf_tree rqt_tf_tree  
    `打开遥控器和运动控制节点，进行运动控制的测试`  
    roslaunch joy_control we_joy_control.launch		//打开遥控器  
    roslaunch base_controller base_controller.launch	//打开底盘控制节点，可以通过遥控器实现运动控制，并获取里程计信息  
    `发布激光雷达安装位置，运行激光雷达数据获取程序以及gmapping程序`  
    roslaunch run_agv run_agv.launch				//发布激光雷达、IMU的安装位置
        设置激光雷达IP：192.168.1.125  255.255.255.0  192.168.1.1
        可通过ping 192.168.1.222 或者ifconfig或者sudo tcpdump -n -i enp0s31f6来检查连接状态
    roslaunch lslidar_n301_decoder lslidar_n301.launch --screen	//开始获取激光雷达数据  
    roslaunch hwtimu hwtimusubexp.launch		//开始获取IMU数据  
    roslaunch set_gmapping robot_gmapping.launch	//通过gmapping进行建图  
    `打开rviz，查看激光雷达的数据，以及建图的结果`  
    rosrun rviz rviz  					//通过rviz查看激光雷达的数据  
    roslaunch set_gmapping save_map_my.launch  		//保存地图  
    `运行move_base，进行导航`  
    roslaunch set_gmapping move_base.launch  		//
