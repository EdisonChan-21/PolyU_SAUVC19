import cv2
import numpy as np
import imutils

def testDetect_quadrant(img):
    location,img = testDetect(img)
    return changeToCartesian(location,img),img


def changeToCartesian(location,img):
    img_height, img_width = img.shape[:2]
    location_c = []
    if not(location == []):
        location_c= (-int((img_width/2) - location[0]),int((img_height/2) - location[1]))
        return location_c
    return location

def changeToPosCoordinate(location,img):
    img_height, img_width = img.shape[:2]
    location_p = []
    if not(location == []):
        location_c= (int(location[0]+(img_width/2)),int((img_height/2) - location[1]))
        return location_c
    return location

def testDetect(img):
    frame = img
    frameSize = [640,480]
    fx = frameSize[0]
    fy = frameSize[1]
    location = []
    image = cv2.resize(frame,(fx,fy),interpolation=cv2.INTER_CUBIC)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([15,60,60])
    upper_yellow = np.array([35,255,255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    kernelOpen=np.ones((5,5))
    kernelClose=np.ones((20,20))
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
    blurred = cv2.GaussianBlur(maskClose, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    for c in cnts:
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]))
        cY = int((M["m01"] / M["m00"]))
        c = c.astype("float")
        c = c.astype("int")
        area = cv2.contourArea(c)
        if area > 5000:
            location = [(cX - (fx/2)),((fy/2) - cY)]
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

    if len(location)>0:
        cv2.circle(image, (int((fx/2) + location[0]),int((fy/2) - location[1])), 7, (0, 255, 0), -1)
        cv2.line(image, (int((fx/2) + location[0]),int((fy/2) - location[1])), (int(fx/2), int(fy/2)), (0, 0, 255), 2)
        location = (int((fx/2) + location[0]),int((fy/2) - location[1]))

    cv2.circle(image, (int(fx/2), int(fy/2)), 7, (255, 0, 0), -1)
    return location,image
