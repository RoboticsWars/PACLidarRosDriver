#include "bufferDecode.h"
#include <string.h>
#include <ros/ros.h>
#include "ioapi.h"

LIDAR_SECTION_ANGLE_S s_section_angle[SECTION_ANGLE_COUNT];

int s_pointCloud2_enable = 0;
int s_laserScan_enable = 1;

ros::Publisher m_output_pointCloud2;
ros::Publisher m_output_lidarScan;
float m_angle_offset = 0;

static const int LIDAR_SCAN_MAX_COUNT = 5760;
static const float LIDAR_SCAN_ANGLE_SCALE = 0.0625;
static const int LINE_POINT_MINI_COUNT = 500;
static const float ANGLE_CIRCLE_CONDITION = 270;
static const float UINTCONVERT = 0.01;

static const size_t packet_size = sizeof(lidar_driver::PointStream().data);
static const size_t BLOCK_COUNT_MAX = sizeof(lidar_driver::PointStream().data)/138;

static float s_angle_duration = 300 ;
const double TIME_FLAG_SCALE = 0.0000001;
static int LINE_POINT_COUNT = 128*1024;

static std::vector<POINT_RAW_S> s_lineRawData;
static int s_lineRawCount = 0 ;

static std::vector<POINT_XYZ_S> s_lineData;
static int s_lineCount = 0 ;
static float s_lastAngle = 0 ;

static float s_range_min = 0 ;
static float s_range_max = 0 ;

//static std::vector<float> s_vangles;
//static std::vector<float> s_hangles;
#define DEG2RAD(x) ((x)*M_PI/180.0)

FILE *logFile = NULL;

static unsigned char s_PackageBuffer[DECODE_BUFFER_SIZE];
static int s_bufferSize = 0 ;
static int s_bufferReadIdx = 0 ;
static int s_bufferWriteIdx = 0 ;
static sensor_msgs::PointCloud2 s_outCloud ;
static sensor_msgs::LaserScan s_scan_msg;
static struct timespec s_timemwait;
static ros::Time s_start_scan_time;

static inline void initLidarScan() {
    memset(&s_lineRawData[0] , 0, sizeof (POINT_RAW_S)*LINE_POINT_COUNT );
}
static inline unsigned int strToList(std::vector<float> &list, std::string str)
{
    unsigned int count = 0;
    string::size_type prev = 0;
    string::size_type pos = 0;
    string tmpValue ;
    list.clear();
    while((pos = str.find_first_of(',', pos))!= string::npos)
    {
        tmpValue =  str.substr(prev, pos - prev);
        list.push_back( atof(tmpValue.c_str()));
        pos++;
        prev = pos;
        count++;
    }
    return count;
}

static inline void push_lidarScan(ros::Time start,int point_count,
                                  double scan_time, bool inverted,
                                  float angle_min, float angle_max,
                                  float range_min, float range_max )
{
    static int scan_count = 0;

    point_count = 5760;
    s_scan_msg.header.stamp = start;
    s_scan_msg.header.frame_id = s_outCloud.header.frame_id;
    scan_count++;

    //    s_scan_msg.angle_min =  DEG2RAD(0);
    //    s_scan_msg.angle_max =  DEG2RAD(360-0.0625);
//    angle_min = 0+m_angle_offset;
//    angle_max = 360+m_angle_offset;
//    if(angle_min>= 360) angle_min = angle_min-360;
//    if(angle_min < 0) angle_min = angle_min+360;
//    if(angle_max>= 360) angle_max = angle_max-360;
//    if(angle_max < 0) angle_max = angle_max+360;

    s_scan_msg.angle_min =  DEG2RAD(0)-M_PI;
    s_scan_msg.angle_max =  DEG2RAD(360-0.0625) - M_PI;

    s_scan_msg.angle_increment = DEG2RAD(0.0625);
    //(s_scan_msg.angle_max - s_scan_msg.angle_min) / (double)(point_count-1);

    s_scan_msg.scan_time = scan_time;
    s_scan_msg.time_increment = scan_time / (double)(point_count-1);

    s_scan_msg.range_min = range_min;
    s_scan_msg.range_max = range_max;

    //ROS_INFO("lidarScan pint count %d angle_min %f  angle_max %f",point_count,angle_min, angle_max);
    s_scan_msg.intensities.resize(point_count);
    s_scan_msg.ranges.resize(point_count);
    float tmpIntent = 0 ;
    if (!inverted) {

        //int offsetIndex = point_count/2 ;???
        int tmpindx = 0;
        for (size_t i = 0; i < point_count; i++) {

            //tmpindx = (i+offsetIndex)%point_count;???
            //float read_value = s_lineRawData[ tmpindx ].distance;
            float read_value = s_lineRawData[ i ].distance;
            if (read_value == 0.0)
                s_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                s_scan_msg.ranges[i] = read_value;

            tmpIntent = s_lineRawData[i].intent/4;
            if(tmpIntent >=  2048) tmpIntent = 2047;
            s_scan_msg.intensities[i] = tmpIntent;
        }


        //        for (size_t i = 0; i < point_count; i++) {
        //            float read_value = s_lineRawData[i].distance;
        //            if (read_value == 0.0)
        //                s_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        //            else
        //                s_scan_msg.ranges[i] = read_value;
        //            s_scan_msg.intensities[i] = s_lineRawData[i].intent ;
        //        }

    } else {
        for (size_t i = 0; i < point_count; i++) {
            float read_value =  s_lineRawData[i].distance;
            if (read_value == 0.0)
                s_scan_msg.ranges[point_count-1-i] = std::numeric_limits<float>::infinity();
            else
                s_scan_msg.ranges[point_count-1-i] = read_value;

            tmpIntent = s_lineRawData[i].intent/4;
            if(tmpIntent >=  2048) tmpIntent = 2047;
            s_scan_msg.intensities[i] = tmpIntent;
        }
    }
    initLidarScan();
    //s_lineRawCount = 0 ;
    //m_output_lidarScan.publish(scan_msg);
}

//static inline int checkFrame_sum(float mtAngle,int mtlaserId,sensor_msgs::PointCloud2 &outCloud) {
//  int rtn = 0 ;
//  static float s_angleSum = 0;
//  float angleDif = 0 ;
//  if(mtlaserId!=0) return rtn ;
//  if(mtAngle > s_lastAngle) {
//    angleDif = mtAngle-s_lastAngle;
//    s_angleSum += angleDif;
//    s_lastAngle = mtAngle;
//  } else {
//    //1: 360-> 0
//    if(s_lastAngle -mtAngle> 300)  {
//      angleDif = 360 - s_lastAngle+mtAngle;
//      s_angleSum += angleDif;
//      s_lastAngle = mtAngle;
//    } else {
//      //nothing
//    }
//  }

//  if(s_angleSum >= s_angle_duration) {
//    outCloud.width = s_lineCount;
//    outCloud.data.resize( outCloud.point_step*outCloud.width  );
//    outCloud.row_step = outCloud.data.size();
//    memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
//    rtn = 1;
//    s_lineCount = 0 ;
//    s_angleSum = 0 ;
//  }
//  return rtn ;
//}

static inline int checkFrame(float mtAngle,int mtlaserId,sensor_msgs::PointCloud2 &outCloud) {
    int rtn = 0 ;
    float angleDif = 0 ;
    //  if( mtlaserId !=0 ) return rtn ;

    angleDif = abs(s_lastAngle - mtAngle);
    if(angleDif >= s_angle_duration) {

        if(s_pointCloud2_enable) {
            outCloud.width = s_lineCount;
            outCloud.data.resize( outCloud.point_step*outCloud.width  );
            outCloud.row_step = outCloud.data.size();
            memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
        }

        //ROS_INFO("mtAngle %f s_lastAngle %f angleDif %f \n ",mtAngle,s_lastAngle,angleDif);
        //        ROS_INFO("publish PointCloud2 %d width %d  point_step %d row_step %d\n",s_pointCloud2_enable,
        //        outCloud.width,outCloud.point_step,outCloud.row_step);
        if(s_laserScan_enable) {
            ros::Time end_scan_time = ros::Time::now();
            double scan_duration = (end_scan_time - s_start_scan_time).toSec() * 1e-3;

            push_lidarScan(s_start_scan_time,s_lineRawCount,
                           scan_duration,false,
                           mtAngle,s_lastAngle,
                           s_range_min,s_range_max );
        }



        s_range_min = 1000;
        s_range_max = 0;
        rtn = 1;
        s_lineCount = 0 ;
        //s_lineRawCount = 0 ;
    }
    s_lastAngle = mtAngle;
    return rtn ;
}

static FILE * fFilePoint = 0;
static char fileName[256];
static int filecount =0;

//static inline void filler_angle_data(float diffAngle, float lastAgle, float curAngle) {
//    POINT_RAW_S tmpRaw ;
//    tmpRaw.distance = 0 ;
//    tmpRaw.intent = 0 ;
//    float tmpAngle = lastAgle ;
//    while (tmpAngle < curAngle) {
//        s_lineRawData[s_lineRawCount] = tmpRaw;
//        s_lineRawCount++;
//        tmpAngle += diffAngle;
//    }

//}

static inline void testOpenFile() {
    if(!fFilePoint) {
        sprintf(fileName,"/home/a/testdata/point%d.txt",filecount++);
        fFilePoint = fopen(fileName,"w+");
    }
}

static inline void testCloseFile() {
    if(fFilePoint) {
        fclose(fFilePoint);
        fFilePoint = 0;
    }
}

static inline void testWriteFile(int index, float dist, float angle, float intent) {
    if(fFilePoint) {
        fprintf(fFilePoint,"%d: %f %f %f\n",index,dist,angle,intent);
        // fprintf(fFilePoint,"%f %f %f %f %f %d\n",tmpXyz.x,tmpXyz.y,tmpXyz.z,tmpXyz.intent,tmpDistance,tmpAngle);
    }
}

static inline int processLidarPackage(Lidar_Data_Package *package, sensor_msgs::PointCloud2 &outCloud)
{
    int rtn = 0;
    float tmpDistance,tmpAngle;
    static float lastAngle = 0, difAngle = 0.12;
    const int CHECK_ANGLE = 5;
    POINT_XYZ_S tmpXyz ;
    POINT_RAW_S tmpRaw ;

    //testOpenFile();
    int pushPoint = 0 ;
    for(int i = LIDAR_PACKAGE_POINT_START_INDEX ; i<= LIDAR_PACKAGE_POINT_STOP_INDEX; i++ ){

        tmpDistance =package->groups[i].A/1000.0 ; //mm->m
        tmpAngle =  package->groups[i].B*LIDAR_DATA_ANGLE_SCALE ;
        tmpXyz.intent = package->groups[i].C ;
        tmpRaw.intent = package->groups[i].C ;
        tmpRaw.distance = tmpDistance;

        calcXyz(tmpDistance,tmpAngle,tmpXyz);
        if ( checkFrame(tmpAngle,0 ,outCloud) ) {
            //testCloseFile();
            //testOpenFile();
            rtn =1 ;
        }
        //testWriteFile(i,tmpDistance,tmpAngle,tmpXyz.intent);

        pushPoint = 0 ;
        if(s_section_angle[0].angle_start <= tmpAngle && tmpAngle <= s_section_angle[0].angle_end) {
            pushPoint=  1;
        } else if(s_section_angle[1].angle_start <= tmpAngle && tmpAngle <= s_section_angle[1].angle_end) {
            pushPoint=  1;
        } else if(s_section_angle[2].angle_start <= tmpAngle && tmpAngle <= s_section_angle[2].angle_end) {
            pushPoint=  1;
        } else if(s_section_angle[3].angle_start <= tmpAngle && tmpAngle <= s_section_angle[3].angle_end) {
            pushPoint=  1;
        } else if(s_section_angle[4].angle_start <= tmpAngle && tmpAngle <= s_section_angle[4].angle_end) {
            pushPoint=  1;
        }

        if(pushPoint) {

            //chekc min max
            if(s_range_min > tmpDistance) s_range_min = tmpDistance;
            if(s_range_max < tmpDistance) s_range_max = tmpDistance;

            if(s_pointCloud2_enable) {
                s_lineData[s_lineCount] = tmpXyz;
                s_lineCount++;
                if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;
            }
        } else {
            tmpRaw.distance = 0 ;
            tmpRaw.intent = 0 ;
        }
        //        float tmpValue = tmpAngle -lastAngle;
        //        if( tmpValue > 5 && tmpValue < 200) {
        //            //ROS_INFO("difAngle %f, lastAngle %f, tmpAngle %f",difAngle,lastAngle,tmpAngle);
        //            filler_angle_data(0.625,225.0,315.0);
        //        } else if(tmpValue < 1 && tmpValue > 0 ){
        //            //difAngle = ( tmpValue+difAngle)/2.0;
        //            difAngle = 0.125;
        //        }

        tmpAngle = tmpAngle+m_angle_offset;
        if(tmpAngle >= 360 ) tmpAngle= tmpAngle-360;
        if(tmpAngle < 0 )   tmpAngle= tmpAngle+360;

        int tmpIndex = tmpAngle/LIDAR_SCAN_ANGLE_SCALE;
        s_lineRawData[tmpIndex] = tmpRaw;
        //s_lineRawCount++;
        //if( s_lineRawCount >= LINE_POINT_COUNT ) s_lineRawCount = LINE_POINT_COUNT-1;

        //lastAngle = tmpAngle ;

    }


    return rtn;
}


///////////////////////////////////////////////////////////
/// \brief SSBufferDec::SSBufferDec
///
///
///
SSBufferDec::SSBufferDec()
{
    pthread_mutex_init(&m_mtx,NULL);
    pthread_cond_init(&m_cond, NULL);
    s_timemwait.tv_nsec = 0 ;
    s_timemwait.tv_sec = 1;
    reset();
}

SSBufferDec::~SSBufferDec()
{

}

//int SSBufferDec::moveWriteIndex(int setpIndex)
//{
//  m_decBuf.bufSize += setpIndex;
////  m_packetSize = setpIndex;
////  m_packetNumber++;
//  m_decBuf.wrHead = (m_decBuf.wrHead+setpIndex)%DECODE_BUFFER_SIZE;
//  return m_decBuf.bufSize ;
//}

//int SSBufferDec::moveReadIndex(int setpIndex)
//{
//  m_decBuf.bufSize -= setpIndex;
//  m_decBuf.rdTail = (m_decBuf.rdTail+setpIndex)%DECODE_BUFFER_SIZE;
//  return m_decBuf.bufSize ;
//}

void SSBufferDec::exit() {
    pthread_mutex_lock(&m_mtx);
    pthread_cond_signal(&m_cond);
    pthread_mutex_unlock(&m_mtx);
}

int SSBufferDec::nextWriteIndex()
{
    m_decBuf.wrHead =( m_decBuf.wrHead +1)% DECODE_BUFFER_COUNT;

    pthread_mutex_lock(&m_mtx);
    pthread_cond_signal(&m_cond);
    pthread_mutex_unlock(&m_mtx);
    return m_decBuf.wrHead;
}

int SSBufferDec::nextReadIndex()
{
    m_decBuf.rdTail =( m_decBuf.rdTail +1)% DECODE_BUFFER_COUNT;
    return m_decBuf.rdTail;
}

unsigned char *SSBufferDec::getWriteIndex()
{
    return (m_decBuf.buffer+(m_decBuf.wrHead*LIDAR_DATA_PACKAGE_SIZE)) ;
}

unsigned char *SSBufferDec::getReadIndex()
{
    return (m_decBuf.buffer+(m_decBuf.rdTail*LIDAR_DATA_PACKAGE_SIZE)) ;
}

int SSBufferDec::isFull()
{
    int isfull = 0 ;
    if( ((m_decBuf.wrHead+1)%DECODE_BUFFER_COUNT) == m_decBuf.rdTail  ) {
        isfull = 1;
    }
    return isfull;
}

int SSBufferDec::isEmpty()
{
    int isempty = 0 ;
    if( m_decBuf.wrHead == m_decBuf.rdTail) {
        isempty = 1;
    }
    return 0;
}

//int SSBufferDec::writeBuffer(unsigned char *data, int size)
//{
//  if(m_decBuf.bufSize+size >= DECODE_BUFFER_SIZE) return 0 ;

//  memcpy(m_decBuf.buffer+m_decBuf.wrHead,data,size);
//  m_decBuf.bufSize += size;
//  m_decBuf.wrHead += size ;

//  //  ROS_INFO_STREAM( "writeBuffer"
//  //                  << " bufSize " << m_decBuf.bufSize
//  //                  << " wrHead "<< m_decBuf.wrHead
//  //                  << " rdTail " <<m_decBuf.rdTail);
//  return size ;
//}

int searchPackage(Lidar_Data_Package **outPackage) {

    int outflag = 0 ;
    Lidar_Data_Package * tmpPackage;
    *outPackage = 0 ;
    while ( s_bufferSize >= sizeof(Lidar_Data_Package) ) {

        tmpPackage =(Lidar_Data_Package *)(s_PackageBuffer+s_bufferReadIdx);

        if( tmpPackage->groups[LIDAR_PACKAGE_HEAD_INDEX].A ==LIDAR_PACKAGE_HEAD_FLAG
                &&  tmpPackage->groups[LIDAR_PACKAGE_TAIL_INDEX].A ==LIDAR_PACKAGE_TAIL_FLAG){

            s_bufferSize -= sizeof(Lidar_Data_Package);
            s_bufferReadIdx += sizeof(Lidar_Data_Package) ;

            outflag = 1;
            *outPackage = tmpPackage;
            return outflag;

        } else {
            s_bufferSize -= 1;
            s_bufferReadIdx +=1 ;
        }
    }//end while

    return outflag;
}


int SSBufferDec::readPacket()
{
    Lidar_Data_Package *outPackage;


    if(m_decBuf.wrHead ==  m_decBuf.rdTail) {
        pthread_mutex_lock(&m_mtx);
        pthread_cond_wait(&m_cond,&m_mtx);
        pthread_mutex_unlock(&m_mtx);
    }
    int ispackage = 0 ;
    if( m_decBuf.wrHead !=  m_decBuf.rdTail ) {
        //ROS_INFO("s_PackageBuffer w %d  r %d \n", s_bufferWriteIdx,s_bufferReadIdx);
        //        printIndex();
        //        saveFrameToFile("/mnt/hgfs/dev/data/readPacket",getReadIndex(),LIDAR_DATA_PACKAGE_SIZE);

        memcpy(s_PackageBuffer+s_bufferWriteIdx, getReadIndex(), LIDAR_DATA_PACKAGE_SIZE );

        s_bufferSize += LIDAR_DATA_PACKAGE_SIZE;
        s_bufferWriteIdx += LIDAR_DATA_PACKAGE_SIZE;

        nextReadIndex();

        //decode one package
        ispackage = searchPackage(&outPackage);
        if( ispackage ) {
            if( processLidarPackage(outPackage,s_outCloud) ) {

                if(s_pointCloud2_enable) {
                    m_output_pointCloud2.publish(s_outCloud);
                }

                if(s_laserScan_enable) {
                    m_output_lidarScan.publish(s_scan_msg);
                }
                SSBufferDec::ResetPointCloud2(s_outCloud);
                s_start_scan_time = ros::Time::now();;
            }
        }

        if( (DECODE_BUFFER_SIZE -s_bufferWriteIdx) <= LIDAR_DATA_PACKAGE_SIZE) {
            bufferReverse();
        }
    }

    return 0;
}

//int SSBufferDec::readPacket(lidar_driver::PointStream &pkt)
//{
//  int rtn = 0;
//  int flag = 0 ;
//  Lidar_Data_Package *tmpPackage;
//  int packageIndex = 0 ;


//  do{
//      packageIndex = m_packetNumber*sizeof(Lidar_Data_Package); //输出缓存的index
//      tmpPackage = (Lidar_Data_Package *)(s_PackageBuffer + packageIndex) ; //输出包地址
//      flag = searchPackageFromBuffer(&m_decBuf,tmpPackage);  //缓存数据流解码包信息，并存入s_PackageBuffer
//      if(flag) {
//          m_packetNumber++;  //数据包个数
//      }
//  }while (flag) ;


//  if(m_packetNumber>0) {
//      pkt.data.resize( m_packetNumber*sizeof(Lidar_Data_Package) );
//      pkt.packetSize = sizeof(Lidar_Data_Package);
//      pkt.packetNumber = m_packetNumber;
//      memcpy(&pkt.data[0], s_PackageBuffer,  m_packetNumber*sizeof(Lidar_Data_Package) );
//      reset();
//      rtn = 1;
//  }

//  //bufferReverse(&m_decBuf);

//  return rtn ;
//}

int SSBufferDec::size()
{
    return m_decBuf.bufSize;
}

int SSBufferDec::freeSize()
{
    return DECODE_BUFFER_SIZE-m_decBuf.bufSize;
}




void SSBufferDec::bufferReverse()
{
    char tmpBuffer[LIDAR_DATA_PACKAGE_SIZE];
    if (s_bufferSize > 0) {
        memcpy(tmpBuffer, s_PackageBuffer+s_bufferReadIdx, s_bufferSize);
        memcpy(s_PackageBuffer, tmpBuffer, s_bufferSize);
        s_bufferReadIdx = 0;
        s_bufferWriteIdx = s_bufferSize;
    } else {
        s_bufferWriteIdx = s_bufferReadIdx = s_bufferSize = 0;
    }
    return;
}

void SSBufferDec::reset()
{
    //memset(&m_packet,0,sizeof(m_packet)) ;
    m_packetSize = 0;
    m_packetNumber = 0 ;
    m_decBuf.bufSize = m_decBuf.rdTail = m_decBuf.wrHead = 0 ;
    bufferReverse();
    SSBufferDec::InitPointcloud2(s_outCloud) ;
}

void SSBufferDec::printIndex()
{
    ROS_INFO(" write %d read %d\n", m_decBuf.wrHead,m_decBuf.rdTail);
}

int SSBufferDec::Depacket(lidar_driver::PointStream &inPack, sensor_msgs::PointCloud2 &outCloud , ros::Publisher &rosOut)
{
    int rtn =0;
    Lidar_Data_Package *package;

    for( int i = 0 ; i < inPack.packetNumber;i++) {
        package = (Lidar_Data_Package*)(&inPack.data[0] + i*inPack.packetSize);
        if( package->groups[LIDAR_PACKAGE_HEAD_INDEX].A ==LIDAR_PACKAGE_HEAD_FLAG
                &&  package->groups[LIDAR_PACKAGE_TAIL_INDEX].A ==LIDAR_PACKAGE_TAIL_FLAG){
            if( processLidarPackage(package,outCloud) ) {

                //              ROS_INFO("publish PointCloud2 width %d  point_step %d row_step %d\n",
                //                       outCloud.width,outCloud.point_step,outCloud.row_step);
                rosOut.publish(outCloud);
                SSBufferDec::ResetPointCloud2(outCloud);
            }
        } else {
            ROS_INFO("Depacket Error \n");
        }

    }
    return rtn ;
}

///
/// \brief SSBufferDec::InitPointcloud2
/// \param initCloud
///
void SSBufferDec::InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) {
    initCloud.data.clear();
    initCloud.is_bigendian = false ;//false;      //stream foramt
    initCloud.fields.resize(4);          //line format
    initCloud.is_dense = false;

    int tmpOffset = 0 ;
    for(int i=0; i < initCloud.fields.size() ;i++) {
        switch(i) { //value type
        case 0:
            initCloud.fields[i].name = "x" ;
            initCloud.fields[i].datatype = 7u;
            break;
        case 1:
            initCloud.fields[i].name = "y" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 2:
            initCloud.fields[i].name = "z" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 3:
            initCloud.fields[i].name = "intensity" ;
            initCloud.fields[i].datatype = 7u;//2u;
            tmpOffset += 4;
            break;
            //    case 4:
            //      initCloud.fields[i].name = "laserid" ;
            //      initCloud.fields[i].datatype = 2u;
            //      tmpOffset += 1;
            //    case 5:
            //      initCloud.fields[i].name = "timeflag" ;
            //      initCloud.fields[i].datatype = 8u;
            //      tmpOffset += 8;
            //      break;
        }
        initCloud.fields[i].offset = tmpOffset ;      //value offset
        initCloud.fields[i].count = 1 ;
    }
    initCloud.height = 1;
    initCloud.point_step = sizeof(POINT_XYZ_S);
    initCloud.width = 0 ;
    //node name
    std::string node_name = ros::this_node::getName();

    std::string frame_id_str = "/world";
    std::string frame_id_path = node_name + "/frame_id";
    ros::param::get(frame_id_path,frame_id_str);

    initCloud.header.frame_id = frame_id_str;

    s_lastAngle = 0 ;
    s_lineData.resize(LINE_POINT_COUNT);
    s_lineRawData.resize(LINE_POINT_COUNT);
    initLidarScan();
    s_lineCount = 0 ;
    s_lineRawCount = 0 ;
}

void SSBufferDec::ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) {
    initCloud.width = 0;
}

void SSBufferDec::SetAngleDuration(float value)
{
    if(value <10  || value > 360)
        return ;
    s_angle_duration = value;
}

