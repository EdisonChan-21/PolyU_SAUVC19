import gateDetector as gDetect
import testDetection as testDetect
import cv2
import numpy as np
import handler
import asyncio
import classifier

def logger(data,log):
    if not (data == [] or data is None):
        logSize = 10
        tempLog = log
        tempLog.append(data)
        if len(tempLog)>logSize:
            del tempLog[0]
        return tempLog
    return log

def pointDrawer(img,log):
    # tempImg = img
    tempLog = []
    for i in log:
        location_p = gDetect.changeToPosCoordinate(i,img)
        cv2.circle(img, location_p, 5, (0, 255, 0), -1)
        tempLog.append([location_p[0],location_p[1]])
    # print(tempLog)
    if not (log == [] or log is None):
        medianPoint = np.median(np.array(tempLog), axis=0)
        # print(medianPoint)
        cv2.circle(img, (int(medianPoint[0]),int(medianPoint[1])), 10, (255, 255, 255), -1)
    return img

def taskTest(control):
    cap = cv2.VideoCapture(0)
    task1Log = []
    while(True):
        ret, frame = cap.read()
        if ret==True:
            location_q,img = testDetect.testDetect_quadrant(frame)
            task1Log = logger(location_q,task1Log)
            classifier.gateLocationClassifier(task1Log)
            handler.gateLocationHandler(classifier.gateLocationClassifier(task1Log),control)
            print(classifier.gateLocationClassifier(task1Log))
            # print(task1Log)
            img = pointDrawer(img,task1Log)
            cv2.imshow('Img',img)
            # img, edges = gate.gateImage(frame)
            # cv2.imshow('Gate',img)
            # cv2.imshow('Edge',edges)
            # await asyncio.sleep(0.0001)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    control.end()

async def task1(control):
    cap = cv2.VideoCapture(0)
    task1Log = []
    while(True):
        ret, frame = cap.read()
        if ret==True:
            location_c,img,edges = gDetect.gateLocation_c(frame)
            task1Log = logger(location_c,task1Log)
            handler.gateLocationHandler(classifier.gateLocationClassifier(task1Log),control)
            # print(task1Log)
            img = pointDrawer(img,task1Log)
            cv2.imshow('Img',img)
            # img, edges = gate.gateImage(frame)
            # cv2.imshow('Gate',img)
            cv2.imshow('Edge',edges)
            # await asyncio.sleep(0.0001)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        await asyncio.sleep(0.0001)
    cap.release()
    cv2.destroyAllWindows()
    control.end()
