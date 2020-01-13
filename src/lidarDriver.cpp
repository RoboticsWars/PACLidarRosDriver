#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "lidarDriver.h"
#include "lidar_driver/LidarCommand.h"

#include <sensor_msgs/PointCloud2.h>

#include "bufferDecode.h"
namespace lidar_driver {

//static const size_t packet_size = sizeof(lidar_driver::PointStream().data);
static const int MESSAGE_PACKET_NUM = 64 ;

static Device_Driver *s_this = NULL;

static char s_simuFileName[] = "test.imp";

void * data_handle_thread_fun(void *data) {
    Device_Driver * dataEngine = (Device_Driver *)data;
    dataEngine->getPackage();
}

bool CommandHandle(lidar_driver::LidarCommand::Request  &req,
                   lidar_driver::LidarCommand::Response &res)
{
    res.status = 1;

    ROS_INFO("request: cmd= %d , speed = %d Hz", (int)req.cmd, (int)req.speed);
    ROS_INFO("sending back response: [%d]", (int)res.status);

    LIDAR_COMMAND_S tmpProg ;
    tmpProg.cmdstat = (LIDAR_CMD_E)req.cmd;
    tmpProg.scnSpeed = req.speed;
    if(s_this) {
        s_this->prog_Set(tmpProg);
    }
    return true;
}

Device_Driver::Device_Driver(ros::NodeHandle mtNode, int type)
{

    bool ok = true;

    memset(s_section_angle,0,sizeof(s_section_angle));
    //配置文件参数读取
    //服务及话题初始化
    std::string filename = s_simuFileName;

    //node name
    std::string node_name = ros::this_node::getName();

    //command name
    std::string command_name = "lidar_control";
    std::string command_path = node_name + "/control_name";
    ros::param::get(command_path,command_name);

    command_path = "lidar_driver/" + command_name;

    m_svr = mtNode.advertiseService(command_path, CommandHandle);  //创建设备控制服务

    //section angle
     std::string section_agl_path;
    char numstr[21];
    for(int i=0; i < SECTION_ANGLE_COUNT; i++) {
         s_section_angle[i].angle_start = 0;
         s_section_angle[i].angle_end = 0 ;
         sprintf(numstr, "/section_agl_start_%d", i+1);
         section_agl_path = node_name + numstr;
         ros::param::get(section_agl_path, s_section_angle[i].angle_start);

         sprintf(numstr, "/section_agl_end_%d", i+1);
         section_agl_path = node_name + numstr;
         ros::param::get(section_agl_path, s_section_angle[i].angle_end);

         ROS_INFO("section %d: angle start %f angle end %f",i,s_section_angle[i].angle_start, s_section_angle[i].angle_end);
    }
    //topic format
    std::string pointCloud2_enable_path = node_name + "/point_cloud2_enable";
    ros::param::get(pointCloud2_enable_path,s_pointCloud2_enable);
    ROS_INFO("s_pointCloud2_enable %d",s_pointCloud2_enable);

    std::string laserScan_enable_path = node_name + "/laser_scan_enable";
    ros::param::get(laserScan_enable_path,s_laserScan_enable);
    ROS_INFO("s_laserScan_enable %d",s_laserScan_enable);


    //advertise name
    std::string advertise_lidarScan_name = "lidarScan";
    std::string advertise_lidarScan_path = node_name + "/advertise_lidarScan_name";
    ros::param::get(advertise_lidarScan_path,advertise_lidarScan_name);
    advertise_lidarScan_path = "lidar_driver/" + advertise_lidarScan_name;
    ROS_INFO("%s : advertise name %s : %s",node_name.c_str(),
             advertise_lidarScan_name.c_str(), advertise_lidarScan_path.c_str() );



    std::string advertise_name = "lidar_points";
    std::string advertise_path = node_name + "/advertise_name";
    ros::param::get(advertise_path,advertise_name);
    advertise_path = "lidar_driver/" + advertise_name;
    ROS_INFO("%s : advertise name %s : %s",node_name.c_str(), advertise_name.c_str(), advertise_path.c_str() );

    //device  ip name
    std::string device_ip = DEVICE_IP_STRING;
    std::string ip_path = node_name + "/device_ip";
    ros::param::get(ip_path,device_ip);
    ROS_INFO("%s : device_ip name %s : %s",node_name.c_str(), device_ip.c_str(), ip_path.c_str() );

    //device port name
    int dataport = DEVICE_PORT_NUMBER;
    std::string port_path = node_name + "/device_port";
    ros::param::get(port_path,dataport);
    ROS_INFO("%s : device_port  %d : %s",node_name.c_str(), dataport, port_path.c_str() );

    //pc port name
    int pcport = PC_PORT_NUMBER;
    std::string pc_port_path = node_name + "/pc_port";
    ros::param::get(pc_port_path,pcport);
    ROS_INFO("%s : pc_port_path  %d : %s",node_name.c_str(), pcport, pc_port_path.c_str() );

    //fps name
    m_fps = CMD_SCAN_SPEED_15HZ;
    std::string speed_path = node_name + "/fps";
    ros::param::get(speed_path,m_fps);
    ROS_INFO("%s : speed_path  %d : %s",node_name.c_str(), m_fps, speed_path.c_str() );

    m_angle_offset = 0 ;
    std::string angle_offset_path = node_name + "/angle_offset";
    ros::param::get(angle_offset_path,m_angle_offset);
    ROS_INFO("%s : speed_path  %d : %s",node_name.c_str(), m_angle_offset, angle_offset_path.c_str() );

    m_command.cmdstat = (LIDAR_CMD_E)1 ;
    m_command.scnSpeed = m_fps;
    //driver init

    if(s_pointCloud2_enable) {
        m_output_pointCloud2 = mtNode.advertise<sensor_msgs::PointCloud2>(advertise_path, MESSAGE_PACKET_NUM);
    }

    if(s_laserScan_enable) {
        m_output_lidarScan = mtNode.advertise<sensor_msgs::LaserScan>(advertise_lidarScan_path, MESSAGE_PACKET_NUM);
    }

    m_devapi = new lidar_driver::TCPSocketAPI(device_ip,dataport,pcport);
    ROS_INFO("lidar_driver:  worker ip %s device port %d  pc port %d", device_ip.c_str(),dataport, pcport);
    pthread_create(&m_dataThread, NULL, data_handle_thread_fun, (void*)this);
    setupLidar();
    s_this = this ;
}

Device_Driver::~Device_Driver()
{
    if(m_devapi) delete m_devapi;
}

int Device_Driver::spinOnce()
{
    int rtn = 0 ;

    if(m_command.cmdstat == eDevCmdWork) {
        rtn = m_devapi->revPacket();
    }

    if(rtn < 0) {
        if(m_command.cmdstat == eDevCmdWork) {
            if( m_devapi->reset() > 0 ) {
                setupLidar();
            }
        }
        return rtn ;
    }
    //m_devapi->getPacket(m_output);
    return rtn ;
}

/** @brief control the device
         *  @param .parameters
         */
int Device_Driver::prog_Set(LIDAR_COMMAND_S &program)
{
    unsigned short tmpcmd = 0 ;

    switch (program.scnSpeed) {
    case CMD_SCAN_SPEED_10HZ:
        tmpcmd =ANGLE_SPEED_10HZ;
        break;
    case CMD_SCAN_SPEED_15HZ:
         tmpcmd =ANGLE_SPEED_15HZ;
        break;
    case CMD_SCAN_SPEED_20HZ:
         tmpcmd =ANGLE_SPEED_20HZ;
        break;
    default:
          tmpcmd =ANGLE_SPEED_20HZ;
        break;
    }
    m_devapi->ioWrite( (unsigned char *)&tmpcmd, sizeof(tmpcmd) );

    switch (program.cmdstat) {
    case eDevCmdIdle:
        tmpcmd =LIDAR_STOP_CMD;
        break;
    case eDevCmdWork:
        tmpcmd = LIDAR_START_CMD;
        break;
    default:
        tmpcmd = LIDAR_START_CMD;
        break;
    }
    m_devapi->ioWrite( (unsigned char *)&tmpcmd, sizeof(tmpcmd) );
    m_command = program;
    return 0;
}

void Device_Driver::getPackage()
{
    while( ros::ok() ) {
        m_devapi->getPacket();
    }
    ROS_INFO("getPackage exit");
}

int Device_Driver::setupLidar()
{
    LIDAR_COMMAND_S tmpProg ;
    tmpProg.cmdstat = eDevCmdWork;
    tmpProg.scnSpeed =  m_fps;
    prog_Set(tmpProg);
}

void Device_Driver::closeLidar()
{
     m_devapi->closeSocket();
}

//end prog_Set

} //lidar_driver namespace
