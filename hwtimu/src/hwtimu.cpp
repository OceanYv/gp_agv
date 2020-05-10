#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <sensor_msgs/Imu.h>
#include "hwtimu/hwtimu.h"

Hwtimu::Hwtimu(ros::NodeHandle nh,ros::NodeHandle nh_pricate):
	node_handle_(nh),
	node_handle_pri_(nh_pricate)
{
  node_handle_pri_.param("com_name",com_name_,std::string("/dev/ttyUSB0"));
  node_handle_pri_.param("baud_rate",baud_rate_,115200);
  node_handle_pri_.param("data_bits",data_bits_,8);
  node_handle_pri_.param("parity",parity_,0);
  node_handle_pri_.param("stop_bits",stop_bits_,1);
  node_handle_pri_.param("mini_data",mini_data_,0);

  node_handle_pri_.param("frame_id",frame_id_,std::string("myHwtimu"));
  node_handle_pri_.param("burst_mode",burst_mode_, false);
  node_handle_pri_.param("loop_rate",loop_rate_,10.0);
  node_handle_pri_.param("imu_topic",imu_topic_,std::string("hwtimu_data"));
  //node_handle_pri_.param("com_file",com_file_,std::string("/home/ubuntu/catkin_tws/src/hwtimu/cfg/com.cfg"));
  //node_handle_pri_.param("com_file",com_file_,std::string("---"));
  node_handle_pri_.param("kind_of_imu",kind_of_imu_,0);//0: hwt,1:adis
  node_handle_pri_.param("is_debug",is_debug_,false);

  ROS_INFO("com_name: %s",com_name_.c_str());
  ROS_INFO("baud_rate: %d",baud_rate_);
  ROS_INFO("data_bits: %d",data_bits_);
  ROS_INFO("parity: %d",parity_);
  ROS_INFO("stop_bits: %d",stop_bits_);
  ROS_INFO("mini_data: %d",mini_data_);

  ROS_INFO("frame_id: %s",frame_id_.c_str());
  ROS_INFO("loop_rate: %6.2f [Hz]",loop_rate_);
  ROS_INFO("burst_mode: %s",(burst_mode_ ? "true": "false"));
  ROS_INFO("imu_topic: %s",imu_topic_.c_str());
  //ROS_INFO("com_file: %s",com_file_.c_str());
  ROS_INFO("kind_of_imu: %d",kind_of_imu_);
  ROS_INFO("is_debug: %s",is_debug_==true? "true":"false");

  memset(accl,0,sizeof(accl));
  memset(gyro,0,sizeof(gyro));
  memset(magn,0,sizeof(magn));
  memset(angl,0,sizeof(angl));
  memset(orten,0,sizeof(orten));

  for(int i=0;i!=10;++i)
  {
     if(!openPort())
     break;
     usleep(10000);
  }
  hwtimu_data_pub_ = node_handle_.advertise
                       <sensor_msgs::Imu>(imu_topic_,100);
  if(kind_of_imu_==0)
      imuRcvThread=new boost::thread(boost::\
                   bind(&Hwtimu::sampleFromPort,this));
  else
      imuRcvThread=new boost::thread(boost::\
                   bind(&Hwtimu::sampleFromPort_lpms,this));
 }

Hwtimu::Hwtimu()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Hwtimu(nh,nh_private);
 }

Hwtimu::~Hwtimu()
{
  if(imuRcvThread)
  {
    imuRcvThread->join();
    delete imuRcvThread;
    imuRcvThread=NULL;
    ROS_INFO("Thread imuRcvThread stopped!");
  }
  HwtimuCom.close();
}

int Hwtimu::publish_imu_data()
{
    sensor_msgs::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    // Linear acceleration
    data.linear_acceleration.x = accl[0];
    data.linear_acceleration.y = accl[1];
    data.linear_acceleration.z = accl[2];

    // Angular velocity (rotatianal velocity)
    data.angular_velocity.x = gyro[0];
    data.angular_velocity.y = gyro[1];
    data.angular_velocity.z = gyro[2];

    // Orientation
    data.orientation.x = orten[1];//q1
    data.orientation.y = orten[2];//q2
    data.orientation.z = orten[3];//q3
    data.orientation.w = orten[0];//q0

	// orientation, angular_velocity, linear_acceleration,
	// all covariance unknown
    for(int i=0;i<9;++i){
        data.orientation_covariance[i]=0;
        data.angular_velocity_covariance[i]=0;
        data.linear_acceleration_covariance[i]=0;
    }

    hwtimu_data_pub_.publish(data);
    if(0==kind_of_imu_)
      ROS_WARN("angl[x,y,z]=[%4.4f,%4.4f,%4.4f]",angl[0],angl[1],angl[2]);
    return 0;
}

int Hwtimu::openPort()
{
  ROS_INFO("---This is Hwtimu::openPort() function---");
  if(HwtimuCom.open(com_name_,baud_rate_,data_bits_,parity_,stop_bits_,mini_data_))
  {
    ROS_INFO("---This is the end of Hwtimu::openPort()---");
  }
  else
  {
    //ROS_ERROR("Open com %s error!",com_name_.c_str());
    ROS_INFO("---This is the end of Hwtimu::openPort()---");
    return -1;
  }
    return 0;
}

/**
 * @brief Change big endian 2(4) byte into short
 * @param data Head pointer to the data
 * @return Converted value
 */
int16_t Hwtimu::big_endian_to_short(unsigned char *data)
{
  unsigned char buff[2] = {data[1], data[0]};
  return *reinterpret_cast<int16_t*>(buff);
}

int16_t Hwtimu::little_endian_to_short(unsigned char *data)
{
  unsigned char buff[2] ={data[0],data[1]};
  return *reinterpret_cast<int16_t*>(buff);
}

double Hwtimu::little_endian_to_double(unsigned char *data)
{
  int16_t int16_buff=little_endian_to_short(data);
  double rtnDouble=(double)(int16_buff);
  return rtnDouble;
}

double Hwtimu::little_endian_to_double_lpms(unsigned char *data)
{
  unsigned char buff[4]={data[0],data[1],data[2],data[3]};
  float transfloat= *reinterpret_cast<float*>(buff);
  double retdouble=(double)transfloat;
  return retdouble;
}

/**
 * @brief when in debug mode, print the data buffer
 * @param data: Head pointer to the data,len:length of the buffer
 * @return 0:success, otherwise false
 */
int Hwtimu::print_data_buffer(unsigned char *data, int len,int printColor=0)
{
  if(printColor==2)
  {
    for(int i=0;i<len;++i){
      ROS_ERROR("databuffer[%d]=%02X",i,data[i]);
    }
  }
  
  else if(printColor==1)
  {
    for(int i=0;i<len;++i){
      ROS_WARN("databuffer[%d]=%02X",i,data[i]);
    }
  }
  
  else
  {
    for(int i=0;i<len;++i){
        ROS_INFO("databuffer[%d]=%02X",i,data[i]);
    }
  }
    return 0;
}

/**
 * @brief Begin to receive data from serial port
 * @param None
 * @return
 */
int Hwtimu::sampleFromPort_lpms()
{
  ROS_INFO("Sample from port(Lpms Format) begin!");
  ros::Rate lpRate(loop_rate_);
  unsigned char recvBuffer[670];
  unsigned char dataBuffer[670];
  size_t size =0;
  size_t len=0;

  int re;
  fd_set  readfds;
  int m_port_file = HwtimuCom.getComHandle();
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 60000;
  int dataLen = 67;
  while(ros::ok()&&loop_rate_)
  {
    while(1)
    {
      FD_ZERO(&readfds);
      FD_SET(m_port_file, &readfds);

      re = select(m_port_file+1, &readfds, NULL, NULL, &tv);
      if(re>0 && FD_ISSET(m_port_file, &readfds))
      {
        size = HwtimuCom.read(recvBuffer, 670);
        memcpy(dataBuffer+len, recvBuffer, size);
        len += size;
        ROS_INFO("Successfully read size:%ld",size);

        if(len >= 5*dataLen)
        {
          if(!truncateData_lpms(dataBuffer,len))
          {
              ROS_INFO("Hwtimu::sampleFromPort:truncateData ok!");
              break;
          }
          ROS_INFO("Hwtimu::sampleFromPort:truncateData failed!");
          if(is_debug_)  print_data_buffer(dataBuffer,len,1);
          len=0;
          memset(dataBuffer,0xff,sizeof(dataBuffer));
          continue;
        }

        else if(len >= dataLen)
        {
          if(!truncateData_lpms(dataBuffer,len))
          {
            ROS_INFO("Hwtimu::sampleFromPort:truncateData ok!");
            break;
          }
          ROS_INFO("Hwtimu::sampleFromPort:truncateData failed!");
          if(is_debug_) print_data_buffer(dataBuffer,len,1);
          continue;
        }
        else{
          ROS_INFO("Data length too short:len=%ld",len);
          continue;
        }
      }

      else if(re == 0)
      {
        //ROS_ERROR("Receive HwtimuCom feedback time out!");
        continue;
      }

      else // re<0
      {
        ROS_ERROR("Select HwtimuCom error!");
        continue;
      }
    } // end of while(1)

    if(dataBuffer[0]!=0x3A || dataBuffer[1]!=0x01 ||
        dataBuffer[2]!=0x00 || dataBuffer[3]!=0x09 ||
        dataBuffer[66]!=0x0A || dataBuffer[65]!=0x0D)
    {
      for(int i=0;i<67;++i)
          ROS_INFO("databuffer[%d]=%02X",i,dataBuffer[i]);
      lpRate.sleep();
      continue;
    }

      // dispose the dataBuffere
      unsigned char singlecmd[67];
      memcpy(singlecmd,&dataBuffer[0],sizeof(singlecmd));
      if(0==resolveSingleCmd_lpms(singlecmd,len)){
           ROS_INFO("Resolve the data buffer success!");
           publish_imu_data();
      }else{
           ROS_INFO("Resolve the data error!");
      }
      memset(dataBuffer,0xff,sizeof(dataBuffer));
      len=0;
      lpRate.sleep();
    }
    ROS_INFO("Sample from port (Lpms Format) stopped!");
    return 0;
}

int Hwtimu::sampleFromPort()
{
  ROS_INFO("Sample from port begin!");
  ros::Rate lpRate(loop_rate_);
  unsigned char recvBuffer[550];
  unsigned char dataBuffer[550];
  size_t size = 0;
  size_t len = 0;

  int re;
  fd_set  readfds;
  int m_port_file = HwtimuCom.getComHandle();
  struct timeval tv;
  int dataLen = 55;
  while(ros::ok()&&loop_rate_)
  {
    while(1)
    {
      FD_ZERO(&readfds);
      FD_SET(m_port_file, &readfds);
      tv.tv_sec = 0;
      tv.tv_usec = 60000;
      re = select(m_port_file+1, &readfds, NULL, NULL, &tv);
      if(re>0 && FD_ISSET(m_port_file, &readfds))
      {
        size = HwtimuCom.read(recvBuffer, sizeof(recvBuffer));
        memcpy(dataBuffer+len, recvBuffer, size);
        len += size;
        ROS_INFO("Successfully read size:%ld",size);

        if(len >= 5*dataLen)
        {
          if(!truncateData(dataBuffer,len))
          {
              ROS_INFO("Hwtimu::sampleFromPort:truncateData ok!");
              break;
          }
          ROS_INFO("Hwtimu::sampleFromPort:truncateData failed!");
          if(is_debug_)  print_data_buffer(dataBuffer,len,1);
          len=0;
          memset(dataBuffer,0xff,sizeof(dataBuffer));
          continue;
        }

        else if(len >= dataLen)
        {
          if(!truncateData(dataBuffer,len))
          {
            ROS_INFO("Hwtimu::sampleFromPort:truncateData ok!");
            break;
          }
          ROS_INFO("Hwtimu::sampleFromPort:truncateData failed!");
          if(is_debug_)  print_data_buffer(dataBuffer,len,1);
          continue;
        }else{
          ROS_INFO("Data length too short:len=%ld",len);
          continue;
        }
      }

      else if(re == 0)
      {
        ROS_ERROR("Receive HwtimuCom feedback time out!");
        break;
      }

      else
      {
        ROS_ERROR("Select HwtimuCom error!");
        break;
      }
    }// end of while(1)

    if(dataBuffer[0]!=0x55 || dataBuffer[1]!=0x51 ||
       dataBuffer[11]!=0x55 || dataBuffer[12]!=0x52 ||
       dataBuffer[22]!=0x55 || dataBuffer[23]!=0x53 ||
       dataBuffer[33]!=0x55 || dataBuffer[34]!=0x54 ||
       dataBuffer[44]!=0x55 || dataBuffer[45]!=0x59)
    {
      lpRate.sleep();
      continue;
    }

    // dispose the dataBuffer
    unsigned char singlecmd[55];
    memcpy(singlecmd,dataBuffer,sizeof(singlecmd));
    if(0==resolveSingleCmd(singlecmd)){
      ROS_INFO("Resolve the data buffer success!");
      publish_imu_data();
    }else{
        ROS_INFO("Resolve the data error!");
    }
    memset(dataBuffer,0xff,sizeof(dataBuffer));
    lpRate.sleep();
  }
  ROS_INFO("Sample from port stopped!");
  return 0;
}

/**
 * @brief Resoleve the 55 bytes stored in the singlecmd
 * @param data  First address of 55 bytes 
 * @retval 0 Success
 * @retval -1 Failure
 */
 int Hwtimu::resolveSingleCmd(unsigned char* data)
 {
   if(!sumCheck(data)) 
   {
     ROS_INFO("Hwtimu::resolveSingleCmd: sumCheck error!");
     return -1;
   }

   for(int i=1;i<=45;i+=11)
   {
      switch (data[i])
      {
        case 0x51:
            accl[0]=little_endian_to_double(&data[i+1])*16.0*9.8/32768.0;
            accl[1]=little_endian_to_double(&data[i+3])*16.0*9.8/32768.0;
            accl[2]=little_endian_to_double(&data[i+5])*16.0*9.8/32768.0;
            break;
        case 0x52:
            gyro[0]=little_endian_to_double(&data[i+1])*2000.0/32768.0;
            gyro[1]=little_endian_to_double(&data[i+3])*2000.0/32768.0;
            gyro[2]=little_endian_to_double(&data[i+5])*2000.0/32768.0;
            break;
      case 0x53:
            angl[0]=little_endian_to_double(&data[i+1])*180.0/32768.0;
            angl[1]=little_endian_to_double(&data[i+3])*180.0/32768.0;
            angl[2]=little_endian_to_double(&data[i+5])*180.0/32768.0;
            break;
      case 0x54:
            magn[0]=little_endian_to_double(&data[i+1]);
            magn[1]=little_endian_to_double(&data[i+3]);
            magn[2]=little_endian_to_double(&data[i+5]);
            break;
      case 0x59:
            orten[0]=little_endian_to_double(&data[i+1])/32768.0;
            orten[1]=little_endian_to_double(&data[i+3])/32768.0;
            orten[2]=little_endian_to_double(&data[i+5])/32768.0;
            orten[3]=little_endian_to_double(&data[i+7])/32768.0;
            break;
      default:
            return -1;
      }
   }
   return 0;
 }

 int Hwtimu::resolveSingleCmd_lpms(unsigned char* data,size_t &length)
 {
   bool check=sumCheck_lpms(data);
   ROS_INFO("Data check successful: checksum=%s,length=%ld",
                           check==true?"true":"false",length);
   if(!check || length!=67) {
     return -1;
   }

   accl[0]=little_endian_to_double_lpms(&data[23]);
   accl[1]=little_endian_to_double_lpms(&data[27]);
   accl[2]=little_endian_to_double_lpms(&data[31]);

   gyro[0]=little_endian_to_double_lpms(&data[11]);
   gyro[1]=little_endian_to_double_lpms(&data[15]);
   gyro[2]=little_endian_to_double_lpms(&data[19]);

   orten[0]=little_endian_to_double_lpms(&data[47]);
   orten[1]=little_endian_to_double_lpms(&data[51]);
   orten[2]=little_endian_to_double_lpms(&data[55]);
   orten[3]=little_endian_to_double_lpms(&data[59]);
   return 0;
 }

/**
 * @brief Truncate the data receive from the serial com
 *        at both the start and the end
 * @param dataBuffer First address of data buffer
 * @param length  Same as len in function sampleFromPort()
 * @retval 0 Success
 * @retval -1 Failure
 */
int Hwtimu::truncateData(unsigned char* dataBuffer,size_t &length)
{
  for(int i=length-11;i>=44;--i){
    if(dataBuffer[i]==0x55 && dataBuffer[i+1]==0x59 &&
       dataBuffer[i-11]==0x55 && dataBuffer[i-10]==0x54 &&
       dataBuffer[i-22]==0x55 && dataBuffer[i-21]==0x53 &&
       dataBuffer[i-33]==0x55 && dataBuffer[i-32]==0x52 &&
       dataBuffer[i-44]==0x55 && dataBuffer[i-43]==0x51){
         length=55;
         int startIndex=i-44;
         memmove(dataBuffer,dataBuffer+startIndex,55);
         return 0;
       }
  }
  return -1;
}

int Hwtimu::truncateData_lpms(unsigned char* dataBuffer,size_t &length)
{
  for(int i=length-1;i>=66;--i)
  {
    if(dataBuffer[i]==0x0A && dataBuffer[i-1]==0x0D &&
       dataBuffer[i-66]==0x3A && dataBuffer[i-65]==0x01 &&
       dataBuffer[i-64]==0x00 && dataBuffer[i-63]==0x09)
    {
      length=67;
      int startIndex=i-66;
      memmove(dataBuffer,dataBuffer+startIndex,67);
      return 0;
    }
  }
  return -1;
}

/**
 * @brief check the sum of the all data 
 * @param data start address of all 11 bytes data
 * @retval flase data is incorrect
 * @retval true data is correct
 */
bool Hwtimu::sumCheck(unsigned char* data)
{
  for(int head=0;head<=44;head=head+11){
    unsigned char checksum=0;
    for(int i=head,cnt=0;cnt<10;++cnt,++i){
      checksum+=data[i];
    }
    if(checksum!=data[head+10]){
      return false;
    }
  }
  return true;
}

bool Hwtimu::sumCheck_lpms(unsigned char* data)
{
  uint16_t checksum=0;
  for(int i=1;i<63;++i)
  {
    checksum=checksum+data[i];
  }
  unsigned char LSB=(unsigned char)(checksum&0x00ff);
  unsigned char MSB=(unsigned char)((checksum&0xff00)>>8);
  if(LSB==data[63] && MSB==data[64])
    return true;
  return false;
}

int Hwtimu::get_product_id(std::string& data)
{
  if(kind_of_imu_==1)
    data=std::string("ADIS16365");
  else
    data=std::string("HWT901");
  return 0;
}

int Hwtimu::update(void)
{
  return 0;
}

int Hwtimu::update_burst(void)
{
  return 0;
}