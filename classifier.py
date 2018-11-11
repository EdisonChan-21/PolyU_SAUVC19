import cv2
import numpy as np
import gateDetector as gDetect


def gateLocationClassifier(log):
    if(len(log)<10):
        return []
    tempLog = []
    for i in log:
        tempLog.append([i[0],i[1]])
    # print(tempLog)
    if not (log == [] or log is None):
        medianPoint = np.median(np.array(tempLog), axis=0)
        # print(medianPoint)
    countX = 0
    countY = 0
    tolerance = 10
    for i in tempLog:
        if(abs(medianPoint[0]-i[0])<tolerance):
            countX+=1
        if(abs(medianPoint[1]-i[1])<tolerance):
            countY+=1
    # print(countX)
    if(countX >=5):
        return medianPoint
    else:
        return []

def nearGateClassifier():
    pass

def flareLocationClassifier():
    pass

def nearFlareClassifier():
    pass
