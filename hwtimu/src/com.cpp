#include <string.h>
#include <hwtimu/com.h>
#include <ros/ros.h>
#include <stdio.h>

CCom::CCom()
{
    m_port_file = -1;
    opened = false;
    inited = false;
}

CCom::CCom(const char *_name, int _speed, int _databits, 
                int _parity, int _stopbits,int _minircv)
{
    name = std::string(_name);
    speed = _speed;
    databits = _databits;
    parity = _parity;
    stopbits = _stopbits;
    minircv = _minircv;

    m_port_file = -1;
    opened = false;
    inited = true;
}

CCom::CCom(const std::string _name, int _speed, int _databits, 
                int _parity, int _stopbits,int _minircv)
{
    name = _name;
    speed = _speed;
    databits = _databits;
    parity = _parity;
    stopbits = _stopbits;
    minircv = _minircv;

    m_port_file = -1;
    opened = false;
    inited = true;
}

CCom::~CCom()
{
    close();
}

bool CCom::OpenPort(const char * portName)
{
    m_port_file = ::open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_port_file == -1)
    {
        opened = false;
        ROS_ERROR("Open port failed:%s",portName);
        return false;
    }

    if(fcntl(m_port_file, F_SETFL, FNDELAY)<0)
	{
		ROS_INFO("fcntl failed!");
		return false;
	}
    opened = true;
    ROS_INFO("Successfully opened:%s",portName);

    struct termios old_options;
	struct termios newtio;
    if (tcgetattr(m_port_file, &old_options) != 0)
    {
        ROS_ERROR("Get port attribute error!");
        return false;
    }
	//115200,8比特数据，１个开始位，１个停止位，无奇偶性，no flow control
	bzero(&newtio,sizeof(newtio));
	/*步骤一，设置字符大小*/
	newtio.c_cflag |= CLOCAL|CREAD;
	newtio.c_cflag &= ~CSIZE;
    
    /*设置数据位*/
    switch(databits){
        case 7:
            newtio.c_cflag |=CS7;
            break;
        case 8:
            newtio.c_cflag |=CS8;
            break;
        default:
            ROS_ERROR("Unsupported data size!");
            return false;
    }

    /*设置奇偶校验位*/
    switch(parity){
        //case 'O': //奇数
        //case 'o':
        case 1:
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |=PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        //case 'E': //偶数
        //case 'e':
        case 2:
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        //case 'N'://无奇偶校验位
        //case 'n':
        case 0:
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            ROS_ERROR("Unsupported parity!");
            return false;
    }

    /*设置波特率*/
    switch(speed){
        case 2400:
            cfsetispeed(&newtio,B2400);
            cfsetospeed(&newtio,B2400);
            break;
        case 4800:
            cfsetispeed(&newtio,B4800);
            cfsetospeed(&newtio,B4800);
            break;
        case 9600:
            cfsetispeed(&newtio,B9600);
            cfsetospeed(&newtio,B9600);
            break;
        case 115200:
            cfsetispeed(&newtio,B115200);
            cfsetospeed(&newtio,B115200);
            break;
        case 460800:
            cfsetispeed(&newtio,B460800);
            cfsetospeed(&newtio,B460800);
            break;
        default:
            ROS_ERROR("Unsupported speed!");
            return false;
    }

    /*设置停止位*/
    switch(stopbits){
        case 1:
            newtio.c_cflag &= ~CSTOPB;
            break;
        case 2:
            newtio.c_cflag |= CSTOPB;
            break;
        default:
            ROS_ERROR("Unsupported speed!");
            return false;
    }
	/*无流控制*/
	newtio.c_cflag &= ~CRTSCTS;
    /*设置等待时间和最小接收字符*/
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = minircv;

    /*处理未接受字符*/
	tcflush(m_port_file,TCIFLUSH);
    /*激活新配置*/
	if(tcsetattr(m_port_file,TCSANOW,&newtio)!=0){
		ROS_INFO("set TCSANOW error");
		return false;
	}
	ROS_INFO("initilize serial success!");
	
    return true;
}

bool CCom::open()
{
    if(opened)
        return true;

    if(!inited)
        return false;

    if (!OpenPort(name.c_str()))
        return false;

    return true;
}

bool CCom::open(const char *_name, int _speed, int _databits,
                int _parity, int _stopbits, int _minircv)
{
    name = std::string(_name);
    speed = _speed;
    databits = _databits;
    parity = _parity;
    stopbits = _stopbits;
    minircv = _minircv;
    inited = true;

    return this->open();
}

bool CCom::open(const std::string _name, int _speed, int _databits,
                int _parity, int _stopbits, int _minircv)
{
    name = std::string(_name);
    speed = _speed;
    databits = _databits;
    parity = _parity;
    stopbits = _stopbits;
    minircv = _minircv;
    inited = true;

    return this->open();
}

size_t CCom::write(byte *data, size_t length)
{
    if(!opened)
        return 0;
    size_t len = 0;
    devdiceLock.lock();
    len = ::write(m_port_file, data, length);
    devdiceLock.unlock();
    if(len == -1)
        ROS_ERROR("Com write failed. Return value: -1");
    return len;
}

size_t CCom::read(void *buf, size_t length)
{
    if(!opened)
        return 0;
    size_t len = 0;
    devdiceLock.lock();
    len = ::read(m_port_file, buf, length);
    devdiceLock.unlock();
    return len;
}

bool CCom::close()
{
    int trytime=0;
    if (opened)
    {
        while(0!=::close(m_port_file) && trytime<10){
            usleep(10000);
            trytime++;
        }
        if(trytime=10){
            ROS_ERROR("CCom::close() failed to close port!");
            return false;
        }
        opened = false;
        ROS_INFO("Close com:%s",name.c_str());
    }
    return true;
}

int CCom::getComHandle()
{
    return m_port_file;
}

void CCom::showData()
{
    struct termios options;
    if (tcgetattr(m_port_file, &options) != 0)
    {
        ROS_ERROR("Get port attribute error!");
        return;
    }
    int len = sizeof(options);
    byte* p = new byte[len];
    memcpy(p, &options, len);
    printf("\n");
    for(int i = 0;  i < len; i++)
        printf("%02x ", p[i]);
    printf("\n");
    return;

    FILE* fp;
    fp = fopen("/home/either/com.cfg", "wb");
    if(fp)
    {
        fwrite(&options, sizeof(options), 1, fp);
        fclose(fp);
    }
    else
         ROS_ERROR("Can't save com setting file");
}

bool CCom::SetPortSpeed(int speed)
{
    struct termios options;
    if (tcgetattr(m_port_file, &options) != 0)
    {
        ROS_ERROR("Get port attribute error!");
        return false;
    }

    unsigned int setSpeed;
    switch (speed)
    {
    case 300:
        setSpeed = B300;
        break;
    case 1200:
        setSpeed = B1200;
        break;
    case 2400:
        setSpeed = B2400;
        break;
    case 4800:
        setSpeed = B4800;
        break;
    case 9600:
        setSpeed = B9600;
        break;
    case 19200:
        setSpeed = B19200;
        break;
	case 38400:
		setSpeed = B38400;
		break;
	case 57600:
		setSpeed = B57600;
		break;
	case 115200:
		setSpeed = B115200;
		break;
    default:
        ROS_ERROR("Error! Baud rate not supported!");
        return false;
    }
    if (cfsetispeed(&options, setSpeed) != 0)
    {
        ROS_ERROR("Error! Set in baud rate failed!");
        return false;
    }
    if (cfsetospeed(&options, setSpeed) != 0)
    {
        ROS_ERROR( "Error! Set out baud rate failed!");
        return false;
    }
    options.c_cflag |= (CLOCAL | CREAD);
    if (tcsetattr(m_port_file, TCSANOW, &options) != 0)
    {
        ROS_ERROR("Error! Set port attribute failed1!");
        return false;
    }
    return true;
}

bool CCom::SetParity(int databits, char parity, int stopbits)
{
    struct termios options;
    if (tcgetattr(m_port_file, &options) != 0)
    {
        ROS_ERROR("Error! Get port attribute failed!");
        return false;
    }
    options.c_cflag &= ~CSIZE;

    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        ROS_ERROR("Unsupported data size!");
        return false;
    }

    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB; // Clear parity enable
        options.c_iflag &= ~INPCK; // Enable parity checking
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB); //Set odd
        options.c_iflag |= INPCK; // Disnable parity checking
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB; // Enable parity
        options.c_cflag &= ~PARODD; // SetEven
        options.c_iflag |= INPCK; //Disnable parity checking
        break;
    case 'S':
    case 's': //as no parity
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        ROS_ERROR("Unsupported parity!");
        return false;
    }

    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        ROS_ERROR("Unsupported stop bits!");
        return false;
    }

    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] =  0;

    tcflush(m_port_file, TCIFLUSH);

    if (tcsetattr(m_port_file, TCSANOW, &options) != 0)
    {
        ROS_ERROR("Error! Set port attribute failed!");
        return false;
    }
    return true;
}

