def createGrid(b):
    q9 = 9
    q4 = 4
    q8 = 8
    for r in range(q9):
        for c in range(q9):
            if (r < q4):
                if (c != q8 and Matrix[r][c + 1] and Matrix[r][c]):
                    b[r * q9 + c][r * q9 + c + 1] = 1
                if (c != 0 and Matrix[r + 1][c] and Matrix[r][c]) and (c >= q4):
                    b[r * q9 + c][(r + 1) * q9 + c] = 1
                if (r != 0 and Matrix[r - 1][c] and Matrix[r][c] and c != q8 and (c <= q4)):
                    b[r * q9 + c][(r - 1) * q9 + c] = 1;
            elif (r == q4):
                if (c == 0):
                    if (Matrix[r][c + 1] and Matrix[r][c]):
                        b[r * q9 + c][(r) * q9 + c + 1] = 1;
                elif (c == q8):
                    if (Matrix[r][c - 1] and Matrix[r][c]):
                        b[r * q9 + c][(r) * q9 + c - 1] = 1;
                else:
                    if (Matrix[r][c - 1] and Matrix[r][c]):
                        b[r * q9 + c][(r) * q9 + c - 1] = 1;
                    if (Matrix[r][c + 1] and Matrix[r][c]):
                        b[r * q9 + c][(r) * q9 + c + 1] = 1;
                if (c < q4 and Matrix[r - 1][c] and Matrix[r][c]):
                    b[r * q9 + c][(r - 1) * q9 + c] = 1
                if (c > q4 and Matrix[r + 1][c] and Matrix[r][c]):
                    b[r * q9 + c][(r + 1) * q9 + c] = 1

            else:
                if (c != 0 and Matrix[r][c - 1] and Matrix[r][c]):
                    b[r * q9 + c][(r) * q9 + c - 1] = 1;
                if (r != q8 and Matrix[r + 1][c] and Matrix[r][c] and c != 0 and c >= q4):
                    b[r * q9 + c][(r + 1) * q9 + c] = 1;
                if (c != q8 and Matrix[r - 1][c] and Matrix[r][c] and c <= q4):
                    b[r * q9 + c][(r - 1) * q9 + c] = 1;
    return b
def coord(i):
    return [int(i % 9), int(i / 9)]
def coordInGraph(i):
    box = coord(i) 
    box = [j*100+50 for j in box]
    return [box[0], box[1]]
def pathDecision(px1, py1):
    vs = np.zeros(81, dtype=bool)
    pr = np.zeros(81, dtype=int)
    for i in range(81):
        vs[i] = False
        pr[i] = -1
    q = []
    q.append(py1 * 9 + px1)
    while True:
        vs[q[0]] = True;
        for i in range(81):
            if (b[q[0]][i] and (not vs[i])):
                vs[i] = True
                pr[i] = q[0]
                q.append(i)
        if (len(q) != 0):
            del q[0]
        if(Matrix[int(q[0] / 9)][q[0] % 9] == z1):
            break
    path = [0, 0]
    x = pr[q[0]];
    path[0] = q[0];
    path[1] = pr[q[0]];
    while (x != -1 and pr[x] != -1):
        path.append(pr[x])
        x = pr[x];
    path.reverse()
    Graph = np.copy(blank_image)
    for i in range(len(path)-1):
        box0 = coordInGraph(path[i])
        box1 = coordInGraph(path[i+1])
        cv2.arrowedLine(Graph, (box0[0],box0[1]), (box1[0],box1[1]), (255,0,0))
        box2 = coord(path[i+1])
        px1 = box2[0]
        py1 = box2[1]
    cv2.circle(Graph , (box1[0],box1[1]), 2, (0,0,255), -1)
    return Graph, px1, py1, path
def LOCOMOTION(coordinatesToSend):
    lastError = 0
    BOX = [25, 25, 75, 75] 
    global frame
    _, frame = cap.read()
    # frame = cv2.imread('imageFromFeed.png')
    frame = frame[roi[0]:roi[1], roi[2]:roi[3]]
    def giveMeBox():
        global frame
        _, frame = cap.read()
        # frame = cv2.imread('imageFromFeed.png')
        frame = frame[roi[0]:roi[1], roi[2]:roi[3]]
        process(frame)
        for line in lines:
            cv2.arrowedLine(frame, (line[0], line[1]), (line[2], line[3]) , (255,0,0) , 2) 
        for c in range(9):
            for r in range(9):
                cv2.circle(frame , (centreCord[c][r][0],centreCord[c][r][1]) , 2, (0,0,255), -1)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            writeToSerial(0, 0, 1, 1)
            exit()
        return BOX
    def process(frame):
        global noBot
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        threshY = cv2.inRange(hsv, BackColor[0], BackColor[1])
        threshR = cv2.inRange(hsv, FrontColor[0], FrontColor[1])
        thresh = cv2.bitwise_or(threshR, threshY)
        img = cv2.bitwise_and(frame, frame, mask=thresh)
        contoursR,hR = cv2.findContours(threshR,1,2)
        contoursY,hY = cv2.findContours(threshY,1,2)

        if(len(contoursY) and len(contoursR)):
            BOX[2], BOX[3], noBot = drawAndCount(contoursR, img, 2)
            BOX[0], BOX[1], noBot = drawAndCount(contoursY, img, 0) 
        # cv2.imshow('threshR',threshR)
        # cv2.imshow('threshY',threshY)
        # cv2.imshow('thresh',thresh)
        # cv2.imshow('imgR',imgR)
        # cv2.imshow('imgY',imgY)
        cv2.imshow('img',img)
    def drawAndCount(contours, img, which):
        Max = 0
        MaxContour = contours[0]
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04*cv2.arcLength(cnt,True), True)
            if len(approx)>=3 and len(approx)<10:
                x, y, w, h = cv2.boundingRect(cnt)
                if(Max<w*h):
                    Max = w*h
                    MaxContour = cnt
        x, y, w, h = cv2.boundingRect(MaxContour)
        cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 1)
        cv2.circle(img,(x+int(w/2),y+int(h/2)), 3, (0,0,255), -1)
        if(Max<thresholdForRect):
            print('insert The Bot','Max is: ',Max)
            return BOX[which], BOX[which+1], True
        else:
            return x+int(w/2), y+int(h/2), False

    def PIDLineError(error, linex, liney, BOX):
        global lastError
        Bx = int( (BOX[0]+BOX[2])/2 )
        By = int( (BOX[1]+BOX[3])/2 )
        livePositionGraph = np.copy(frame)
        cv2.arrowedLine(livePositionGraph, (linex, liney), (Bx, By) , (0,255,0) , 2)
        cv2.arrowedLine(livePositionGraph, (BOX[0], BOX[1]), (BOX[2], BOX[3]) , (0,0,255) , 2)
        cv2.imshow('livePositionGraph', livePositionGraph)
        motorSpeed = error*Kp + Kd*(error-lastError);
        motorSpeed = int(motorSpeed)
        global rightMotorSpeed
        global leftMotorSpeed
        rightMotorSpeed = BaseSpeedLine + motorSpeed;
        leftMotorSpeed = BaseSpeedLine - motorSpeed;
        if (rightMotorSpeed > MaxSpeedLine ): rightMotorSpeed = MaxSpeedLine
        if (leftMotorSpeed > MaxSpeedLine ): leftMotorSpeed = MaxSpeedLine
        if (rightMotorSpeed < 0): rightMotorSpeed = 0
        if (leftMotorSpeed < 0): leftMotorSpeed = 0
        print('L:',leftMotorSpeed,'R:',rightMotorSpeed)
        writeToSerial(leftMotorSpeed, rightMotorSpeed, 1, 1)
        lastError = error
    def moveWrtLine(line , BOX):
        Bx = int( (BOX[0]+BOX[2])/2 )
        By = int( (BOX[1]+BOX[3])/2 )
        #Right is +ve error
        # Horizontal Line
        if((line[1]-line[3])<15):
            if(line[0] < line[2]):
                while(Bx < line[2]):
                    error = By - line[1]
                    PIDLineError(error, Bx, line[1], BOX)
                    BOX = giveMeBox()
                    Bx = int( (BOX[0]+BOX[2])/2 )
                    By = int( (BOX[1]+BOX[3])/2 )

            elif(line[0] > line[2]):
                while(Bx > line[2]):
                    error = line[1] - By
                    PIDLineError(error, Bx, line[1], BOX)
                    BOX = giveMeBox()
                    Bx = int( (BOX[0]+BOX[2])/2 )
                    By = int( (BOX[1]+BOX[3])/2 )

        # Vertical Line
        if(line[0]-line[2]<15):
            if(line[1] < line[3]):
                while(By < line[3]):
                    error = line[0] - Bx
                    PIDLineError(error, line[0], By, BOX)
                    BOX = giveMeBox()
                    Bx = int( (BOX[0]+BOX[2])/2 )
                    By = int( (BOX[1]+BOX[3])/2 )

            elif(line[1] > line[3]):
                while(By > line[3]):
                    error = Bx - line[0]
                    PIDLineError(error, line[0], By, BOX)
                    BOX = giveMeBox()
                    Bx = int( (BOX[0]+BOX[2])/2 )
                    By = int( (BOX[1]+BOX[3])/2 )

        #Instantaneos Stop
        writeToSerial(leftMotorSpeed, rightMotorSpeed, 0, 0)
        sleep(instantStopLine)

    def calAngle(line, BOX, livePositionGraph):
        a = [BOX[2]-BOX[0],BOX[3]-BOX[1]]
        b = [line[2]-line[0], line[3]-line[1]]
        if(a[0]==0 and a[1]==0):
            return 1, livePositionGraph
        sign = (a[0]*b[1]-a[1]*b[0])/(sqrt(a[0]**2+a[1]**2)*sqrt(b[0]**2+b[1]**2))#vector Product
        mag = (a[0]*b[0]+a[1]*b[1])/(sqrt(a[0]**2+a[1]**2)*sqrt(b[0]**2+b[1]**2))#scalar product
        mag = acos(mag)
        if(sign>0):mag = -mag
        # cv2.arrowedLine(livePositionGraph, (175, 175), (a[0]+175, a[1]+175) , (0,0,255) , 2)
        # cv2.arrowedLine(livePositionGraph, (175, 175), (int(b[0]/(sqrt(b[0]**2+b[1]**2))*50+175), int(b[1]/(sqrt(b[0]**2+b[1]**2))*50+175)) , (255,0,0) , 2)
        return degrees(mag), livePositionGraph
    def moveAngle(line, BOX):
        #Right is +ve error
        livePositionGraph = np.copy(frame)
        angle, livePositionGraph= calAngle(line, BOX, livePositionGraph)
        speed = 0
        while (abs(angle)>20):
            livePositionGraph = np.copy(frame)
            cv2.arrowedLine(livePositionGraph, (BOX[0], BOX[1]), (BOX[2], BOX[3]) , (0,0,255) , 2)
            angle, livePositionGraph = calAngle(line, BOX, livePositionGraph)
            cv2.imshow('livePositionGraph', livePositionGraph)
            speed = 80
            print('speed in turning:', speed, 'angle:', angle)
            if(angle>0):
                writeToSerial(speed, speed, 0, 1)
            elif(angle<0):
                writeToSerial(speed, speed, 1, 0)
            BOX = giveMeBox()

        #Instantaneos Stop
        if(angle>0):
            writeToSerial(speed, speed, 1, 0)
            sleep(instantStopAngle)
            writeToSerial(0, 0, 1, 0)
        elif(angle<0):
            writeToSerial(speed, speed, 0, 1)
            sleep(instantStopAngle)
            writeToSerial(0, 0, 0, 1)

    lines = np.copy(coordinatesToSend)
    for i in range(len(lines)):
        line = lines[i]
        lines[i] = [ centreCord[line[0]][line[1]][0], centreCord[line[0]][line[1]][1], centreCord[line[2]][line[3]][0], centreCord[line[2]][line[3]][1] ]

    lines = lines.astype(np.int16)
    frame = frame.astype(np.uint16)
    for i in range(len(lines)):
        moveAngle(lines[i], giveMeBox())
        moveWrtLine(lines[i], giveMeBox())
    writeToSerial(0, 0, 1, 1)
    cv2.destroyAllWindows()
def writeToSerial(a, b, c, d):
    global noBot
    if(noBot==True):
        ser.write(str.encode('0 0 1 1 0\n'))
        ser.readline()
    else:
        ser.write(str.encode(str(a)+' '+str(b)+' '+str(c)+' '+str(d)+' 0\n'))
        ser.readline()
    pass

import serial
from time import sleep
from math import degrees, sqrt, acos, ceil
import numpy as np
from queue import Queue
import cv2

Matrix = np.load('array.npy')
centreCord = np.load('centreCord.npy')
roi = np.load('roi.npy')
print(Matrix)

ser = serial.Serial("/dev/ttyACM0", 9600)
sleep(2)
print("connection Est");
cap = cv2.VideoCapture(1)
cap.set(3, 720)
cap.set(4, 1280)

Kp = 7
Kd = 70
MaxSpeedLine = 150
BaseSpeedLine = 100
instantStopAngle = 0.250
instantStopLine = 0.100

thresholdForRect = 200

#Initializatins
lastError = 0
noBot = False
rightMotorSpeed=0
leftMotorSpeed=0

FrontColor=np.array([ [90, 60, 135], [115, 125, 225] ])
BackColor=np.array([ [145, 34, 167], [170, 75, 207] ])

b = np.zeros((81, 81))
b = createGrid(b)

start = int(input('Enter 1 or 2 or 3 or 4:'))
b[38][39]=b[39][38]=b[22][31]=b[31][22]=b[41][42]=b[42][41]=b[58][49]=b[49][58]=0
if(start == 1):
    posX=0
    posY=4
    b[38][39]=b[39][38]=1
    b[36][37]=b[37][36]=0 
elif(start == 2):
    posX=4
    posY=0
    b[22][31]=b[31][22]=1
    b[4][13]=b[13][4]=0
elif(start == 3):
    posX=8
    posY=4
    b[41][42]=b[42][41]=1
    b[44][43]=b[43][44]=0
elif(start == 4):
    posX=4
    posY=8
    b[58][49]=b[49][58]=1
    b[67][76]=b[76][67]=0

while True:
    blank_image = np.zeros((900, 900, 3), np.uint8)
    for i in range(81):
        for j in range(81):
            if (b[i][j] == 1):
                box0 = coordInGraph(i)
                box1 = coordInGraph(j)
                cv2.arrowedLine(blank_image, (box0[0], box0[1]), (box1[0], box1[1]), (i*2,255-j*2,i+j))
    cv2.imshow('Img', blank_image)
    cv2.waitKey(1)
    if ( (posX==3 and posY==4) or (posX==4 and posY ==3) or (posX==5 and posY==4) or (posX==4 and posY==5) ):
        z1 = 5
    else:
        z1 = int(input('Enter Your Destination:'))
    Graph, posX, posY, path = pathDecision(posX, posY)
    arrowPath = []
    whichWasSame = 'None'
    for i in range(1, len(path)):
        box = coord(path[i])
        if(coord(path[i-1])[0] == box[0] and whichWasSame != 'X'):
            whichWasSame = 'X'
            arrowPath.append(coord(path[i-1]))
        if(coord(path[i-1])[1] == box[1] and whichWasSame != 'Y'):
            whichWasSame = 'Y'
            arrowPath.append(coord(path[i-1]))
    arrowPath.append(coord(path[len(path)-1]))# adding final position of the bot

    coordinatesToSend = []
    for i in range(1,len(arrowPath)):
        coordinatesToSend.append(arrowPath[i-1])
        coordinatesToSend.append(arrowPath[i])
    coordinatesToSend=np.array(coordinatesToSend)
    coordinatesToSend = coordinatesToSend.reshape(-1 ,4)
    print(coordinatesToSend)
    cv2.imshow('Img', Graph)
    cv2.waitKey(1)
    LOCOMOTION(coordinatesToSend)
    b[36][37]=b[37][36]=b[4][13]=b[13][4]=b[44][43]=b[43][44]=b[67][76]=b[76][67]=1
    if(start == 1):
        b[37][36]=b[38][37]=b[39][38]=0 
        b[27][36]=b[36][27]=b[29][38]=b[38][29]=0
    elif(start == 2):
        b[13][4]=b[22][13]=b[31][22]=0 
        b[22][23]=b[23][22]=b[4][5]=b[5][4]=0
    elif(start == 3):
        b[41][42]=b[42][43]=b[43][44]=0 
        b[44][53]=b[53][44]=b[42][51]=b[51][42]=0
    elif(start == 4):
        b[67][76]=b[58][67]=b[49][58]=0 
        b[76][75]=b[75][76]=b[58][57]=b[57][58]=0

    if(z1 ==5):
        ser.write(str.encode('0 0 1 1 1\n'))
        ser.readline()


cv2.destroyAllWindows()
