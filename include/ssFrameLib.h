#ifndef _FRAME_LIB_H
#define _FRAME_LIB_H
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;
#define CMD_SCAN_SPEED_10HZ  10
#define CMD_SCAN_SPEED_15HZ  15
#define CMD_SCAN_SPEED_20HZ  20

static int ANGLE_SPEED_10HZ = 0x0a50;//0x500a;
static int ANGLE_SPEED_15HZ = 0x0f50;//0x500f;
static int ANGLE_SPEED_20HZ = 0x1450;//0x5014;

static int LIDAR_START_CMD = 0x6273;//0x7362;
static int LIDAR_STOP_CMD   = 0x706f;//0x6F70;

static unsigned short DEVICE_PORT_NUMBER = 5000;
static unsigned short PC_PORT_NUMBER = 6000;

static  std::string DEVICE_IP_STRING = "192.168.31.13";

static unsigned short UDP_FRAME_MIN = 18;
#pragma pack(1)
typedef enum {
    eDevCmdIdle = 0,
    eDevCmdWork,
    eDevCmdSimu,
    eDevCmdBreak,
    eDevCmdReset,
    eDevCmdAsk,
} LIDAR_CMD_E;

typedef struct {
    LIDAR_CMD_E cmdstat;
    int scnSpeed;
}LIDAR_COMMAND_S;

typedef struct {
    float x,y,z ;

    float intent;
    //unsigned char laserid;
    //double timeflag;
    //float angle, range ;
}POINT_XYZ_S;

typedef struct {
    float distance ; //m
    float intent;
}POINT_RAW_S;

#define LIDAR_DATA_ANGLE_SCALE (0.0625)
typedef struct {
    unsigned short A; //: head or tail flag; Distance, mm
    unsigned short B; //: angle, 0.0625,
    unsigned short C; //: gray
}Lidar_Data_Group;

#define LIDAR_DATA_GROUP_MAX (1020)

#define LIDAR_PACKAGE_POINT_START_INDEX (1)
#define LIDAR_PACKAGE_POINT_STOP_INDEX (1000)

#define LIDAR_PACKAGE_HEAD_INDEX (0)
#define LIDAR_PACKAGE_TAIL_INDEX (1019)

#define LIDAR_PACKAGE_HEAD_FLAG (0x0010) //(0x1000)   //
#define LIDAR_PACKAGE_TAIL_FLAG (0X8CA0) //(0xa08c)

#define LIDAR_DATA_PACKAGE_SIZE (LIDAR_DATA_GROUP_MAX*6)
typedef struct {
    Lidar_Data_Group groups[LIDAR_DATA_GROUP_MAX];
}Lidar_Data_Package;

#define DECODE_BUFFER_COUNT (32)
#define DECODE_BUFFER_SIZE  (LIDAR_DATA_PACKAGE_SIZE*DECODE_BUFFER_COUNT) //

typedef struct {
    int wrHead, rdTail, bufSize;
    unsigned char buffer[DECODE_BUFFER_SIZE];
} UDP_DECBUFFER_S;

#define SECTION_ANGLE_COUNT (5)
typedef struct {
    float angle_start;
    float angle_end;
}LIDAR_SECTION_ANGLE_S;

#pragma pack()

#ifdef __cplusplus
extern "C"
{
#endif
int swapchar( unsigned char * _data, int size_ ) ;
int checkSum(unsigned char * _dataBuf, int count_ ) ;
int searchPackageFromBuffer(UDP_DECBUFFER_S *buffer,Lidar_Data_Package *outPackage);
void bufferReverse(UDP_DECBUFFER_S *buffer);
int calcXyz(float &mtRange, float &mtAngle, POINT_XYZ_S &outXyz);
struct timeval  timer_operate(int cmd, char *msg,int idx);
void saveFrameToFile(char *path, unsigned char *data,int size);
#ifdef __cplusplus
}
#endif


#endif
