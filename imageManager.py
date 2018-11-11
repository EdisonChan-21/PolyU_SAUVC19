from gateDetector import gateDetect
import gateDetector
import testDetection as testDetect
import cv2
import numpy as np
import handler
import asyncio
import classifier
import imutils

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
    task1Log = []

    while(True):
        ret, frame = cap.read()
        if ret==True:
            location_c,img,edges = gDetect.gateDetection(frame)
            handler.gateLocationHandler(classifier.gateLocationClassifier(gDetect.getcPointLog()),control)
            box = classifier.boxMedianClassifier(gDetect.getBoxLog())

            img = pointDrawer(img,gDetect.getcPointLog())
            if not (box == []):
                cv2.rectangle(img,
                        (int(box[0]), int(box[1])),
                        (int(box[2]), int(box[3])),
                        [0,0,255], 3)
            cv2.imshow('Img',img)
            cv2.imshow('Edge',edges)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        await asyncio.sleep(0.0001)
    cap.release()
    cv2.destroyAllWindows()
    control.end()
