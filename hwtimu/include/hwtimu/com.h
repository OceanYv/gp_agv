#ifndef ___COM____H____
#define ___COM____H____

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>

typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned char byte;
typedef unsigned char uchar;

#define ByteCast(x) ((byte)(x))

class CCom
{
public:
    CCom();
    CCom(const char * _name, int _speed, int _databits,
        int _parity, int _stopbits,int _minircv);
    CCom(const std::string _name, int _speed,
         int _databits, int _parity, int _stopbits,int _minircv);
    ~CCom();
    bool open();
    bool open(const std::string _name, int _speed, int _databits,
        int _parity, int _stopbits, int _minircv);
    bool open(const char * _name, int _speed, int _databits,
        int _parity, int _stopbits, int _minircv);
    bool close();
    size_t write(byte* data, size_t length);
    size_t read(void *buf, size_t length);
    int getComHandle();
    void showData();

private:
    bool OpenPort(const char * portName);
    bool SetPortSpeed(int speed);
    bool SetParity(int databits, char parity, int stopbits);
    bool opened;
    boost::mutex devdiceLock;
    std::string name;
    int minircv;
    int speed;
    int databits;
    int parity;
    int stopbits;
    int m_port_file;
    bool inited;
};

#endif //___COM____H____

