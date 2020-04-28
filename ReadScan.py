#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import sys
import time
import math

from sensor_msgs.msg import LaserScan

angle = 0
angleNum = 1

beams = 5760 
stop = 0
start = 0
rangeMax = 0.0
rangeMin = float('inf')
times = -1
totalTimes=-1
effectiveCnt = 0
rangeTotal = 0.0
rangeEver = 0
isFirst = True

if len(sys.argv) == 1:
    print("Need arguement:angle, [angle number=1, times=-1]")
    exit(-1)
elif len(sys.argv) >= 2:
    angle = float(sys.argv[1])

if len(sys.argv) >= 3:
    angleNum = float(sys.argv[2])
if len(sys.argv) >= 4:
    times = int(sys.argv[3])
    totalTimes = int(sys.argv[3])

print("Angle:"+str(angle))
print("Angle Number:"+str(angleNum))
print("Total Times:"+str(times))
print("***********Waiting**********")

def scanCallBack(data):
    global angle
    global angleNum
    global beams
    global stop
    global start
    global rangeMax
    global rangeMin
    global times
    global effectiveCnt
    global rangeTotal
    global rangeEver
    global isFirst
    global totalTimes
    
    if isFirst:
        beams = len(data.ranges)
        start  = int(beams/360*angle)
        stop = int(start+(beams/360)*angleNum)
        isFirst = False
    rangeList=[]
    if times>0:
        for i in data.ranges[start:stop]:
            if i <= 60.0:
                rangeList.append(i)
                rangeTotal+=i
        if len(rangeList)==0:
            print("Range List is empty!")
            return
        effectiveCnt += len(rangeList)
        curMax = max(rangeList)
        curMin = min(rangeList)
        if rangeMax < curMax:
            rangeMax = curMax
        if rangeMin > curMin:
            rangeMin = curMin
        if times==1:
            print("Max:"+str(rangeMax))
            print("Min:"+str(rangeMin))
            print("Average:"+str(rangeTotal/effectiveCnt))
            totalNum = totalTimes*angleNum*(beams/360)
            print("Effective Points Number:"+str(effectiveCnt)+"\tTotal Points Number:"+str(totalNum))
            print("Effective Rate:"+str(float(effectiveCnt)/totalNum*100)+"%")
            print("************Done************")
    if times < 0:
        rangeList = data.ranges[start:stop]
        result = ""
        for i in rangeList:
            result+=str(i)
            result+=','
        sys.stdout.write("\r\x1b[KRange List:[" + result +"]\n")
        sys.stdout.flush()
    elif times == 0:
        return
    times-=1



if __name__ == "__main__":
    rospy.init_node("ScanReader",anonymous=False)
    rospy.Subscriber("/scan",LaserScan,scanCallBack,queue_size=1)
    rospy.spin()