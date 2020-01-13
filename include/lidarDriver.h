#ifndef _LIDAR_DRIVER_H
#define _LIDAR_DRIVER_H
#include <ros/ros.h>
#include "ioapi.h"
#include <pthread.h>

namespace lidar_driver
{
class Device_Driver
{
public:
  Device_Driver(ros::NodeHandle mtNode, int type=0);
  ~Device_Driver();

  int spinOnce();
  int prog_Set(LIDAR_COMMAND_S &program);

  void getPackage();
  int setupLidar();
  void closeLidar();
private:
  lidar_driver::TCPSocketAPI *m_devapi;

  ros::ServiceServer m_svr ;
 // lidar_driver::PointStream tmpPacket;
  LIDAR_COMMAND_S m_command ;
  pthread_t m_dataThread;
  int m_fps;

};

}

#endif
