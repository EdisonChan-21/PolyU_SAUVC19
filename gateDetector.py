import cv2
import numpy as np
import math
import colorsys
import imutils
import constant

vSlopeLimit = 3
hSlopeLimit = 5
gap = 10
sideRatio = 0.5
aspectRatioTarget = 1
aspectRatioTolerance = 0.5
cannyThreshold = 800
angleDifference = 5 #degree
vlineMax = 50
hlineMax = 50
centerPoint = None

def line_intersect(line1, line2):
    xdiff = (line1[0] - line1[2], line2[0] - line2[2])
    ydiff = (line1[1] - line1[3], line2[1] - line2[3])
    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return False, False
    else:
        d = (det(*((line1[0],line1[2]), (line1[1],line1[3]))), det(*((line2[0],line2[2]), (line2[1],line2[3]))))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        if( ((line1[0]<=x<=line1[2]) or (line1[2]<=x<=line1[0])) and ((line2[0]<=x<=line2[2]) or (line2[2]<=x<=line2[0])) and (line1[1]<=y<=line1[3]) or (line1[3]<=y<=line1[1]) and ((line2[1]<=y<=line2[3]) or (line2[3]<=y<=line2[1]))):
            return x, y
        else:
            if(lineLength((line1[0],line1[1],line2[0],line2[1]))<gap) or (lineLength((line1[2],line1[3],line2[2],line2[3]))<gap) or (lineLength((line1[0],line1[1],line2[2],line2[3]))<gap) or (lineLength((line1[2],line1[3],line2[0],line2[1]))<gap):
                return x, y
            return False, False

def midpoint(p1, p2):
    return ((abs(p1[0]+p2[0])/2,abs(p1[1]+p2[1])/2))

def findTop(line):
    if(line[1]<line[3]):
        return (line[0],line[1])
    else:
        return (line[2],line[3])

def findBottom(line):
    if(line[1]>line[3]):
        return (line[0],line[1])
    else:
        return (line[2],line[3])

def lineLength(line):
    length = math.sqrt((line[0] - line[2])**2 + (line[1] - line[3])**2)
    return length

def verticalLink(vLine1,hLine1,vLine2):

    midpoint_x, midpoint_y = midpoint((hLine1[0],hLine1[1]),(hLine1[2],hLine1[3]))
    min_x = min(vLine1[0],vLine1[2],vLine2[0],vLine2[2])
    max_x = max(vLine1[0],vLine1[2],vLine2[0],vLine2[2])
    if(min_x<=midpoint_x<=max_x):
        return True
    else:
        return False

def overlap(vLine1,hLine1,vLine2):

    overlap1 = line_intersect(vLine1,hLine1)
    overlap2 = line_intersect(vLine2,hLine1)
    if not(overlap1[0] and overlap1[1] and overlap2[0] and overlap2[1]):
        return False
    else:
        return True

def vLineOnTop(vLine1,hLine1,vLine2):
    vLine1Intersect_y = line_intersect(vLine1,hLine1)[1]
    vLine2Intersect_y = line_intersect(vLine2,hLine1)[1]
    if((findTop(vLine1)[1]+(abs(vLine1[1]-vLine1[3])*0.2))>=vLine1Intersect_y>=findTop(vLine1)[1]) and ((findTop(vLine2)[1]+(abs(vLine2[1]-vLine2[3])*0.2))>=vLine2Intersect_y>=findTop(vLine2)[1]):
        return True
    return False

def hLineOnSide(vLine1,hLine1,vLine2):
    if(sideRatio<=(abs(line_intersect(vLine1,hLine1)[0]-line_intersect(vLine2,hLine1)[0])/lineLength(hLine1))<(1+sideRatio)):
        return True
    else:
        return False

def aspectRatio(vLine1,hLine1,vLine2):
    return (lineLength(hLine1)/((lineLength(vLine1)+lineLength(vLine2))*0.5))

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx,array[idx]

def findCenterPoint(vLine1,hLine1,vLine2):

    vLine1Intersect = line_intersect(vLine1,hLine1)
    vLine2Intersect = line_intersect(vLine2,hLine1)
    if(vLine1Intersect):
        pt1=(int(vLine1Intersect[0]),int(vLine1Intersect[1]))
    else:
        pt1 = (int(findTop(vLine1)[0]),int(findTop(vLine1)[1]))
    pt3 = (int(findBottom(vLine1)[0]),int(findBottom(vLine1)[1]))
    if(vLine2Intersect):
        pt2 = (int(vLine2Intersect[0]),int(vLine2Intersect[1]))
    else:
        pt2 = (int(findTop(vLine2)[0]),int(findTop(vLine2)[1]))
    pt4 = (int(findBottom(vLine2)[0]),int(findBottom(vLine2)[1]))

    centerPoint = (int((pt1[0]+pt2[0]+pt3[0]+pt4[0])/4),int((pt1[1]+pt2[1]+pt3[1]+pt4[1])/4))

    return centerPoint

def findvAngle(l):
    slope = (l[1]-l[3])/(l[0]-l[2])
    radian = math.atan(slope)
    angle = radian*180/math.pi
    return angle

def findhAngle(l):
    slope = (l[0]-l[2])/(l[1]-l[3])
    radian = math.atan(slope)
    angle = radian*180/math.pi
    return angle

def angleDiff(l1,l2):
    sumAngle = l1+l2
    if(sumAngle>90):
        return(abs(l1-l2))
    elif(sumAngle<-90):
        return (l1+l2+180)
    else:
        return(l1+l2)

def gateImage(img):
    location,gate_img,gate_edges = gateDetection(img)
    return gate_img,gate_edges

def gateLocation(img):
    return gateDetection(img)[0]

def changeToCartesian(location,img):
    img_height, img_width = img.shape[:2]
    location_c = []
    if not(location == [] or location is None ):
        location_c= (-int((img_width/2) - location[0]),int((img_height/2) - location[1]))
        return location_c
    return location

def changeToPosCoordinate(location,img):
    img_height, img_width = img.shape[:2]
    location_c = []
    if not(location == [] or location is None ):
        location_c = (int(location[0]+(img_width/2)),int((img_height/2) - location[1]))
        return location_c
    return location

def gateLocation_c(img):
    location,img,edges = gateDetection(img)
    return changeToCartesian(location,img),img,edges

def gateDetection(img):
    vLine = []
    hLine = []
    line_combination = []
    aspectRatioArray = []
    img = imutils.resize(img, width=constant.resizeWidth)
    edges = cv2.Canny(img,cannyThreshold,cannyThreshold,apertureSize = 5)
    lines = cv2.HoughLinesP(edges,1,np.pi/180,50,minLineLength = 10,maxLineGap =50)
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            if not (l[0]-l[2]) ==0:
                slope = abs((l[1]-l[3])/(l[0]-l[2]))
            else:
                slope = math.inf
            if slope > vSlopeLimit:
                vLine.append(l)
            elif slope < hSlopeLimit:
                hLine.append(l)
    filtered_line_combination = []
    # print(len(vLine),len(hLine))
    if not (len(vLine)>vlineMax or len(hLine) > hlineMax):
        for j in hLine:
            for idx_i,i in enumerate(vLine):
                for idx_k,k in enumerate(vLine):
                    if(idx_k>idx_i):
                        if(verticalLink(i,j,k)):
                            # cv2.line(org_img,(i[0], i[1]), (i[2], i[3]),(0,255,255),1)
                            # cv2.line(org_img,(j[0], j[1]), (j[2], j[3]),(0,255,255),1)
                            # cv2.line(org_img,(k[0], k[1]), (k[2], k[3]),(0,255,255),1)
                            # line_combination.append([i,j,k])
                            if overlap(i,j,k):
                                # cv2.line(org_img,(i[0], i[1]), (i[2], i[3]),(0,0,255),1)
                                # cv2.line(org_img,(j[0], j[1]), (j[2], j[3]),(0,0,255),1)
                                # cv2.line(org_img,(k[0], k[1]), (k[2], k[3]),(0,0,255),1)
                                if vLineOnTop(i,j,k):
                                    # cv2.line(org_img,(i[0], i[1]), (i[2], i[3]),(0,0,255),1)
                                    # cv2.line(org_img,(j[0], j[1]), (j[2], j[3]),(0,0,255),1)
                                    # cv2.line(org_img,(k[0], k[1]), (k[2], k[3]),(0,0,255),1)
                                    if hLineOnSide(i,j,k):
                                        # cv2.line(org_img,(i[0], i[1]), (i[2], i[3]),(0,255,255),1)
                                        # cv2.line(org_img,(j[0], j[1]), (j[2], j[3]),(0,255,255),1)
                                        # cv2.line(org_img,(k[0], k[1]), (k[2], k[3]),(0,255,255),1)
                                        if angleDiff(findvAngle(i),findhAngle(j))<angleDifference and angleDiff(findvAngle(k),findhAngle(j))<angleDifference:
                                            # cv2.line(img,(i[0], i[1]), (i[2], i[3]),(0,255,255),1)
                                            # cv2.line(img,(j[0], j[1]), (j[2], j[3]),(0,255,255),1)
                                            # cv2.line(img,(k[0], k[1]), (k[2], k[3]),(0,255,255),1)
                                            filtered_line_combination.append([i,j,k])
                                            aspectRatioArray.append(aspectRatio(i,j,k))
        if not aspectRatioArray == []:
            i,j,k = filtered_line_combination[find_nearest(aspectRatioArray,aspectRatioTarget)[0]]
            # print(find_nearest(aspectRatioArray,aspectRatioTarget)[1])
            if((aspectRatioTarget-aspectRatioTolerance)<=find_nearest(aspectRatioArray,aspectRatioTarget)[1]<=(aspectRatioTarget+aspectRatioTolerance)):
                cv2.line(img,(i[0], i[1]), (i[2], i[3]),(0,255,255),1)
                cv2.line(img,(j[0], j[1]), (j[2], j[3]),(0,255,255),1)
                cv2.line(img,(k[0], k[1]), (k[2], k[3]),(0,255,255),1)
                centerPoint = findCenterPoint(i,j,k)
                cv2.circle(img,centerPoint, 10, (255, 0, 0), -1)
                # print("Center of gate: ", centerPoint)
                return centerPoint,img,edges
    return None,img,edges


if __name__ == "__main__":
    org_img = cv2.imread('../Opencv/Img/Gate4.png')
    # org_img = cv2.imread('../Opencv/Img/Capture5.jpg')
    # org_img = cv2.imread('../Opencv/Img/gate.jpeg')
    # org_img = cv2.imread('gate1.png')
    # org_img = cv2.imread('2.jpg')
    location, img,edges = gateLocation_c(org_img)
    print(location)
    print(changeToPosCoordinate(location,img))
    cv2.circle(img,changeToPosCoordinate(location,img), 10, (255, 0, 0), -1)
    cv2.imshow('Gate',img)
    cv2.imshow('Edge',edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
