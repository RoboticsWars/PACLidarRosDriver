#ifndef _BUFFER_DECODER_H_
#define _BUFFER_DECODER_H_
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "lidar_driver/PointStream.h"
#include "ssFrameLib.h"
#include <ros/ros.h>

extern LIDAR_SECTION_ANGLE_S s_section_angle[SECTION_ANGLE_COUNT] ;
extern int  s_pointCloud2_enable ;
extern int  s_laserScan_enable ;
extern ros::Publisher m_output_pointCloud2;
extern ros::Publisher m_output_lidarScan;

typedef enum {
    eReady,
    eSearch,
    eReadPackData,
    eReadFinish
}DEC_STSTUS_E ;

class SSBufferDec {
public:
    SSBufferDec();
    ~SSBufferDec();
    //  int moveWriteIndex(int setpIndex);
    //  int moveReadIndex(int setpIndex);

    int nextWriteIndex();
    int nextReadIndex();
    unsigned char * getWriteIndex();
    unsigned char * getReadIndex();

    int isFull();
    int isEmpty();
    //int writeBuffer(unsigned char *data, int size);
    int readPacket();
    //int readPacket(lidar_driver::PointStream &pkt);

    int size();
    int freeSize();
    void reset();
    void printIndex();
    int getUdpCount() { return m_packetNumber;}
    int getUdpSize() {return m_packetSize;}

    static int Depacket(lidar_driver::PointStream &inPack, sensor_msgs::PointCloud2 &outCloud , ros::Publisher &rosOut);
    static void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) ;
    static void ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) ;

    static void SetAngleDuration(float value);
    void exit();

private:
    int readBuffer() ;
    int readBufferSteam() ;
    void bufferReverse();
    //int ouputPacket(lidar_driver::PointStream &pkt);

private:
    UDP_DECBUFFER_S m_decBuf;
    //DEC_STSTUS_E m_status ;
    lidar_driver::PointStream m_packet ;
    int m_packetSize ;
    //int m_blockCout ;
    //float s_preAngle ;
    //int m_packetSize;
    int m_packetNumber;
    pthread_mutex_t m_mtx ;
    pthread_cond_t m_cond ;
};


#endif
