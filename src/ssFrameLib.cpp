#include "ssFrameLib.h"
#include "string.h"
#include <math.h>


int swapchar( unsigned char * _data, int size_ ) {
    int i = 0 , j = 0;
   unsigned char tmp = 0 ;
    for ( i=0, j= size_ - 1; i < size_ / 2 ; i++ , j--){
        tmp = _data[i] ;
        _data[i] = _data[j];
        _data[j] = tmp ;
    }
    return 0;
}

int checkSum(unsigned char * _dataBuf, int count_ ) {
    int rtn = 0 ;
    for( int i = 0 ; i < count_ ; i++ ) {
        rtn += _dataBuf[ i ] ;
    }
    rtn = rtn & 0xFF ;
    return rtn ;
}

int searchPackageFromBuffer(UDP_DECBUFFER_S *buffer,Lidar_Data_Package *outPackage) {

    int outflag = 0 ;
    Lidar_Data_Package * tmpPackage;

    while ( buffer->bufSize >= sizeof(Lidar_Data_Package) ) {

        tmpPackage =(Lidar_Data_Package *)(&buffer->buffer[buffer->rdTail]);

        if( tmpPackage->groups[LIDAR_PACKAGE_HEAD_INDEX].A ==LIDAR_PACKAGE_HEAD_FLAG
                &&  tmpPackage->groups[LIDAR_PACKAGE_TAIL_INDEX].A ==LIDAR_PACKAGE_TAIL_FLAG){

            memcpy(outPackage, tmpPackage, sizeof(Lidar_Data_Package));

            buffer->rdTail += sizeof(Lidar_Data_Package);
            buffer->bufSize -= sizeof(Lidar_Data_Package);
            outflag = 1;
            return outflag;
        } else {
            buffer->rdTail += 1;
            buffer->bufSize -= 1;
        }
    }//end while

    return outflag;
}

static char s_tmpBuffer[DECODE_BUFFER_SIZE];
void bufferReverse(UDP_DECBUFFER_S *buffer)
{
  if (buffer->bufSize > 0) {
    memcpy(s_tmpBuffer, buffer->buffer + buffer->rdTail, buffer->bufSize);
    memcpy(buffer->buffer, s_tmpBuffer, buffer->bufSize);
    buffer->rdTail = 0;
    buffer->wrHead = buffer->bufSize;
  } else {
    buffer->wrHead = buffer->bufSize = buffer->rdTail = 0;
  }
  return;
}

int calcXyz(float &mtRange, float &mtAngle, POINT_XYZ_S &outXyz) {
  int rtn = 1;
  double tmptheta=0;//竖直角度
  double ot = 0 ;

  if(mtAngle>360) mtAngle -= 360;
  if(mtAngle<0) mtAngle +=360 ;

  ot = mtAngle*M_PI / 180.0; //水平角度
  //VLP 16 Sensor Coordinate System
  outXyz.x = mtRange * cos(tmptheta) * sin(-ot);
  outXyz.y = mtRange * cos(tmptheta) * cos(-ot);
  outXyz.z = mtRange * sin(tmptheta);

  return rtn ;
}

int time_substract(struct timeval *result, struct timeval *begin,struct timeval *end)
{
    if(begin->tv_sec > end->tv_sec)    return -1;
    if((begin->tv_sec == end->tv_sec) && (begin->tv_usec > end->tv_usec))    return -2;

    result->tv_sec    = (end->tv_sec - begin->tv_sec);

    result->tv_usec    = (end->tv_usec - begin->tv_usec);

    if(result->tv_usec < 0)
    {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }
    return 0;

}

static struct timeval start[20],stop[20],diff[20];

struct timeval  timer_operate(int cmd, char *msg,int idx) {
    if(cmd) {
        memset(&start[idx],0,sizeof(struct timeval));
        memset(&stop[idx],0,sizeof(struct timeval));
        memset(&diff[idx],0,sizeof(struct timeval));
        gettimeofday(&start[idx],0);
    } else {
        gettimeofday(&stop[idx],0);
        time_substract(&diff[idx],&start[idx],&stop[idx]);
        if(msg)
            ROS_INFO("idex %d %s : %d s,%d us\n",idx,msg,(int)diff[idx].tv_sec,(int)diff[idx].tv_usec);
    }
    return diff[idx];
}

static FILE * testPacket = 0 ;
static int filecount = 0 ;
void saveFrameToFile(char *path,unsigned char *data, int size)
{
    char filename[256];
    sprintf(filename,"%s_%d.bin",path,filecount++);
    testPacket = fopen(filename,"wb+");
    if(testPacket) {
        fwrite(data,1,size,testPacket);
        fclose(testPacket);
    }

}
