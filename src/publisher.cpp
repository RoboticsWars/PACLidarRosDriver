#include <ros/ros.h>
#include "lidarDriver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_driver");
  ros::NodeHandle node;
  lidar_driver::Device_Driver * p_driver;
  if(argc >1 ) {
    p_driver = new lidar_driver::Device_Driver(node,1);
  } else {
    p_driver = new lidar_driver::Device_Driver(node,0);
  }


  while( ros::ok() ) {  //loop until shut down
    p_driver->spinOnce();
    ros::spinOnce();
  }

  p_driver->closeLidar();
  delete p_driver;
  return 0;
}
