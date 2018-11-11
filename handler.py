import testDetection as testDetect
import cv2
import numpy as np
import constant

def gateLocationHandler(location_c,control):
    if not(location_c==[] or location_c is None):
        if(location_c[0]>constant.centerPointMargin):
            control.rcLateral = int(control.rcLateralTrim + control.marginRCinput + (location_c[0]/constant.resizeWidth)*control.PWMdiff*control.gain)
        elif(location_c[0]<-constant.centerPointMargin):
            control.rcLateral = int(control.rcLateralTrim - control.marginRCinput + (location_c[0]/constant.resizeWidth)*control.PWMdiff*control.gain)
        else:
            control.rcLateral = control.rcLateralTrim
            control.rcForward = int(control.rcForwardTrim + control.PWMdiff*control.gain)
    else:
        control.resetRcInput()
        gateMissHandler()

def gateMissHandler():
    pass

def nearGateHandler():
    pass

def searchGateHandler():
    pass

def passGateHandler():
    pass

def flareLocationHandler():
    pass

def searchFlareHandler():
    pass

def flareMissHandler():
    pass

def nearFlareHangler():
    pass

def flareDropHandler():
    pass
