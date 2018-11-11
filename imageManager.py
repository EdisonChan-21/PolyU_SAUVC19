from gateDetector import gateDetect
import gateDetector
import testDetection as testDetect
import cv2
import numpy as np
import handler
import asyncio
import classifier
import imutils
import os
import time
import sys


basedir = os.path.dirname(__file__)
sys.path.append(os.path.abspath(os.path.join(basedir, os.path.pardir)))
import ObjTrack.demo.webcam_demo as webcam_demo
import tracker.re3_tracker as re3_tracker

def pointDrawer(img,log):
    tempLog = []
    for location_p in log:
        cv2.circle(img, (location_p[0],location_p[1]), 5, (0, 255, 0), -1)
        tempLog.append([location_p[0],location_p[1]])
    # print(tempLog)
    if not (log == [] or log is None):
        medianPoint = np.median(np.array(tempLog), axis=0)
        # print(medianPoint)
        cv2.circle(img, (int(medianPoint[0]),int(medianPoint[1])), 10, (255, 255, 255), -1)
    return img

async def task1(control):
    cap = cv2.VideoCapture(0)
    gDetect = gateDetect()
    tracker = re3_tracker.Re3Tracker()
    box =[]
    initialize = True
    enlarge = 15
    while(True):
        ret, frame = cap.read()
        if ret==True:
            if (box == []):
                location_c,img,edges = gDetect.gateDetection(frame)
                handler.gateLocationHandler(classifier.gateLocationClassifier(gDetect.getcPointLog()),control)
                box = classifier.boxMedianClassifier(gDetect.getBoxLog())
                img = pointDrawer(img,gDetect.getcPointLog())
                cv2.imshow('Img',img)
                cv2.imshow('Edge',edges)
            else:
                img = frame.copy
                if initialize:
                    box = [box[0]-enlarge,box[1]-enlarge,box[2]+enlarge,box[3]+enlarge]
                    outputBoxToDraw = tracker.track('webcam', frame[:,:,::-1], box)
                    initialize = False
                else:
                    outputBoxToDraw = tracker.track('webcam', frame[:,:,::-1])
                cv2.rectangle(frame,
                        (int(outputBoxToDraw[0]), int(outputBoxToDraw[1])),
                        (int(outputBoxToDraw[2]), int(outputBoxToDraw[3])),
                        [0,0,255], 4)
                cv2.circle(frame, (int((outputBoxToDraw[0]+outputBoxToDraw[2])/2), int((outputBoxToDraw[1]+outputBoxToDraw[3])/2)), 10, [255,255,255], 4)
                cv2.imshow('Img',frame)
                cv2.imshow('Edge',edges)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        await asyncio.sleep(0.0001)
    cap.release()
    cv2.destroyAllWindows()
    control.End()
