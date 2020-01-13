#ifndef __IOAPI_H
#define __IOAPI_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>

#include "ssFrameLib.h"

#include	<arpa/inet.h>	/* inet(3) functions */
#include	<errno.h>
#include	<fcntl.h>		/* for nonblocking */
#include	<netdb.h>
#include	<signal.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<sys/stat.h>	/* for S_xxx file mode constants */
#include	<sys/uio.h>		/* for iovec{} and readv/writev */
#include	<unistd.h>
#include	<sys/wait.h>
#include	<sys/un.h>		/* for Unix domain sockets */

#include "lidar_driver/PointStream.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#define	SA	struct sockaddr

class SSBufferDec;
namespace lidar_driver
{

class IOAPI
{
public:
  IOAPI() ;
  ~IOAPI();

  virtual int ioWrite(unsigned char *data, int size) = 0;
  virtual int ioRead(unsigned char *data, int size) = 0;
  virtual int reset() = 0;
  virtual int revPacket() ;
  virtual int getPacket( ) ;
protected:
  SSBufferDec *m_bufferPro ;
};

////////////////////////////////////////////////////////////////////////
// UDPSocketAPI class implementation
////////////////////////////////////////////////////////////////////////
class UDPSocketAPI : public IOAPI
{
public:
  UDPSocketAPI( std::string ipstr = DEVICE_IP_STRING,
               uint16_t devport   = DEVICE_PORT_NUMBER,
               int16_t pcport     = PC_PORT_NUMBER);
  virtual ~UDPSocketAPI();
  virtual int ioWrite(unsigned char *data, int size) ;
  virtual int ioRead(unsigned char *data, int size) ;
  virtual int reset() ;
private:
  int m_sockfd;
  in_addr devip_;
  sockaddr_in m_devaddr;

  ros::NodeHandle private_nh_;
  uint16_t m_devport_;
  uint16_t m_pcport_;
  std::string devip_str_;
};

////////////////////////////////////////////////////////////////////////
// TCPSocketAPI class implementation
////////////////////////////////////////////////////////////////////////
class TCPSocketAPI : public IOAPI
{
public:
  TCPSocketAPI( std::string ipstr = DEVICE_IP_STRING,
               uint16_t devport   = DEVICE_PORT_NUMBER,
               int16_t pcport     = PC_PORT_NUMBER);
  virtual ~TCPSocketAPI();
  virtual int ioWrite(unsigned char *data, int size) ;
  virtual int ioRead(unsigned char *data, int size) ;
  virtual int reset() ;
  int TCPFcntl(int fd, int cmd, int arg);
  int connect_nonb(int sockfd, const SA *saptr, socklen_t salen, int nsec);
  int tcpAccept();
  void closeSocket();
private:
  int m_sockfd;
  int m_clientSocket;
  in_addr devip;
  struct sockaddr_in m_servaddr;

  ros::NodeHandle private_nh;
  uint16_t m_devport;
  uint16_t m_pcport;
  std::string m_devip_str;
};

} // lidar_driver namespace

#endif
