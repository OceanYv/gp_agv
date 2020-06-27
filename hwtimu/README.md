#cfg下有两个yaml文件
    hwtcfg文件定义了串口配置、坐标系名称、话题名等；
    hwtsubcfg文件定义了话题名；
#launch文件打开了hwtimu文件和hwtimuSubExample文件

#include下有两个头文件
    com文件定义了CCom类，与IMU串口有关:
        三种类型的初始化函数，一个析构函数；
        三种类型的open函数，一个close函数；
        write函数、read函数，getComHandle函数，showData函数；
        还有私有函数：OpenPort、SetPortSpeed、SetParity；
    hwtimu文件定义了Hwtimu类，
        定义了CCom的对象HwtimuCom；
        两种初始化函数（读取yaml文件参数），一个析构函数；
        int publish_imu_data();
            发布imu的消息，类型为sensor_msgs::Imu；
            只填充了线加速度、角速度和方位，三者的协方差都填充为0；
        int sampleFromPort();
        int sampleFromPort_lpms();
        int resolveSingleCmd(unsigned char* data);
        int resolveSingleCmd_lpms(unsigned char* data,size_t &length);
        bool sumCheck(unsigned char* data);
        bool sumCheck_lpms(unsigned char* data);
        int truncateData(unsigned char* dataBuffer,size_t &length);
        int truncateData_lpms(unsigned char* dataBuffer,size_t &length);
        int16_t big_endian_to_short(unsigned char *data);
        int16_t little_endian_to_short(unsigned char *data);
        double little_endian_to_double(unsigned char *data);
        double little_endian_to_double_lpms(unsigned char *data);
        int print_data_buffer(unsigned char* data,int len,int printColor);
        int openPort();
        void closePort();
        int update(void);
        int update_burst(void);
        int read_register(char address, int16_t& data);
        int get_product_id(std::string& data);

    