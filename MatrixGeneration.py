def process(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    threshY = cv2.inRange(hsv, Y[0], Y[1])
    threshR = cv2.inRange(hsv, R[0], R[1])
    thresh = cv2.bitwise_or(threshR, threshY)
    imgY = cv2.bitwise_and(frame, frame, mask=threshY)
    imgR = cv2.bitwise_and(frame, frame, mask=threshR)
    img = cv2.bitwise_and(frame, frame, mask=thresh)

    contoursR,hR = cv2.findContours(threshR,1,2)
    contoursY,hY = cv2.findContours(threshY,1,2)
    contours,h = cv2.findContours(thresh,1,2)

    coordinateArray = []
    colorArray = []
    Sum = 0
    Sum += drawAndCount(coordinateArray, colorArray, contoursR, imgR, 'R')
    Sum += drawAndCount(coordinateArray, colorArray, contoursY, imgY, 'Y') 
    coordinateArray, colorArray = removeSmallContours(coordinateArray, colorArray, Sum)
    coordinateArray = np.array(coordinateArray)
    colorArray = np.array(colorArray)

    # cv2.imshow('threshR',threshR)
    # cv2.imshow('threshY',threshY)
    # cv2.imshow('thresh',thresh)
    cv2.imshow('imgR',imgR)
    cv2.imshow('imgY',imgY)
    # cv2.imshow('img',img)
    cv2.imshow('org', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return coordinateArray, colorArray

#   APPROX POLY DP  
# def drawAndCount(coordinateArray, colorArray, contours, img, color):
#     Sum = 0
#     for cnt in contours:
#         approx = cv2.approxPolyDP(cnt, 0.04*cv2.arcLength(cnt,True), True)
#         # print (len(approx))
#         if len(approx)==3:
#             # print (len(approx))
#             # print ("triangle")
#             x, y, w, h = cv2.boundingRect(cnt)
#             colorArray.append(color+'1')
#             coordinateArray.append([x, y, w, h])
#             cv2.drawContours(img,[cnt], -1, (255, 0, 0), 2)
#             Sum+=w*h
#         elif len(approx)==4:
#             # print (len(approx))
#             x, y, w, h = cv2.boundingRect(cnt)
#             colorArray.append(color+'2')
#             coordinateArray.append([x, y, w, h])
#             cv2.drawContours(img,[cnt], -1, (0, 255, 0), 2)
#             Sum+=w*h
#         elif len(approx)>4 and len(approx)<10:
#             # print (len(approx))
#             # print ("circle")
#             x, y, w, h = cv2.boundingRect(cnt)
#             colorArray.append(color+'3')
#             coordinateArray.append([x, y, w, h])
#             cv2.drawContours(img,[cnt], -1, (255, 255, 0), 2)
#             Sum+=w*h
#     return Sum


# AREA RATIO METHOD
def drawAndCount(coordinateArray, colorArray, contours, img, color):
    Sum = 0
    for cnt in contours:
        rect = cv2.minAreaRect(cnt)
        if((rect[1][0]*rect[1][1])==0):
            continue
        approx = cv2.contourArea(cnt) / (rect[1][0]*rect[1][1])
        if approx >= 0.84:
            print (approx, 'square')
            x, y, w, h = cv2.boundingRect(cnt)
            colorArray.append(color+'2')
            coordinateArray.append([x, y, w, h])
            cv2.drawContours(img,[cnt], -1, (255, 0, 0), 2)
            Sum+=w*h
        elif approx >= 0.7:
            print (approx, 'circle')
            x, y, w, h = cv2.boundingRect(cnt)
            colorArray.append(color+'3')
            coordinateArray.append([x, y, w, h])
            cv2.drawContours(img,[cnt], -1, (0, 255, 0), 2)
            Sum+=w*h
        elif approx >= 0.5:
            print (approx, 'triangle')
            x, y, w, h = cv2.boundingRect(cnt)
            colorArray.append(color+'1')
            coordinateArray.append([x, y, w, h])
            cv2.drawContours(img,[cnt], -1, (255, 255, 0), 2)
            Sum+=w*h
    return Sum


def removeSmallContours(coordinateArray, colorArray, Sum):
    avg = Sum/((9*9)-29)
    i = 0 
    while i<len(coordinateArray):
        w = coordinateArray[i][2]
        h = coordinateArray[i][3]
        if(w*h<avg*0.5):
            del coordinateArray[i]
            del colorArray[i]
            i-=1
        i+=1    
    return coordinateArray, colorArray

import numpy as np
import cv2
cap = cv2.VideoCapture(1)
cap.set(3, 720)
cap.set(4, 1280)

for i in range(10):
    _, frame = cap.read()
    # cv2.imwrite('imageFromFeed.png', frame)    
    # frame = cv2.imread('imageFromFeed.png')
    # frame = cv2.imread('ARENA.jpeg')

roi = cv2.selectROI(frame)
frame = frame[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]

cv2.imshow('frame', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()

Y=np.array([ [20, 100, 130], [35, 255, 255] ])
R=np.array([ [0, 125, 125], [10, 255, 200] ])


coordinateArray, colorArray = process(frame)
blank_image = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)

centres = []
for i in coordinateArray:
    cv2.rectangle(blank_image ,(i[0], i[1]), (i[0]+i[2], i[1]+i[3]), (0,255,0), 1)
    rx = int(i[0]+i[2]/2)
    ry = int(i[1]+i[3]/2)
    centres.append([ rx, ry])
    cv2.circle(blank_image ,(rx, ry) , 1, (255,0,0), 2)
centres = np.array(centres)

box = [min(coordinateArray[:,0]), min(coordinateArray[:,1]), max(coordinateArray[:,0]+coordinateArray[:,2]), max(coordinateArray[:,1]+coordinateArray[:,3])]
cv2.rectangle(blank_image ,(box[0], box[1]), (box[2], box[3]), (0,255,0), 1)

print('No of coordinates detected = ', len(coordinateArray))
cv2.imshow('hello', blank_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

centreCord = np.zeros((9, 9, 2) , dtype=int)
Arrays = np.zeros((9, 9), dtype=int)
for i in range(len(coordinateArray)):
    c = int( (centres[i][0]-box[0])/int((box[2]-box[0])/9) )
    r = int( (centres[i][1]-box[1])/int((box[3]-box[1])/9) )
    centreCord[c][r][0] = centres[i][0]
    centreCord[c][r][1] = centres[i][1]
    dikt = {
        'R1':71,
        'R2':72,
        'R3':73,
        'Y1':81,
        'Y2':82,
        'Y3':83
    }
    Arrays[r][c]=dikt[colorArray[i]]


Arrays[0][4] = Arrays[4][0] = Arrays[8][4] = Arrays[4][8] =1
Arrays[4][4] = 5


#centre
centreCord[4][4] =[int((centreCord[4][3][0]+centreCord[4][5][0])/2) , int((centreCord[4][3][1]+centreCord[4][5][1])/2) ]

#1
centreCord[0][4] =[int((centreCord[0][3][0]+centreCord[0][5][0])/2) , int((centreCord[0][3][1]+centreCord[0][5][1])/2) ]

#2
centreCord[4][0] =[int((centreCord[3][0][0]+centreCord[5][0][0])/2) , int((centreCord[3][0][1]+centreCord[5][0][1])/2) ]

#3
centreCord[8][4] =[int((centreCord[8][3][0]+centreCord[8][5][0])/2) , int((centreCord[8][3][1]+centreCord[8][5][1])/2) ]

#4
centreCord[4][8] =[int((centreCord[3][8][0]+centreCord[5][8][0])/2) , int((centreCord[3][8][1]+centreCord[5][8][1])/2) ]


roi = np.array([ int(roi[1]) , int(roi[1]+roi[3]) , int(roi[0]) , int(roi[0]+roi[2]) ])

print(centreCord)
print(Arrays)
print(roi)

np.save('array.npy', Arrays)
np.save('centreCord.npy', centreCord)
np.save('roi.npy', roi)