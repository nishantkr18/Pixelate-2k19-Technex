import numpy as np
import cv2
cap = cv2.VideoCapture(1)
cap.set(3, 720)
cap.set(4, 1280)

#this is for yellow and red
# A=np.array([ [20, 100, 130], [35, 255, 255] ])
# A=np.array([ [0, 125, 125], [10, 255, 200] ])

# this is for blue and pink
# A=np.array([ [90, 60, 135], [115, 125, 225] ])
A=np.array([ [145, 34, 167], [170, 75, 207] ])


def nothing(x):
    pass

cv2.namedWindow("Trackbars") 
cv2.createTrackbar("L - H", "Trackbars", A[0][0], 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", A[0][1], 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", A[0][2], 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", A[1][0], 255, nothing)
cv2.createTrackbar("U - S", "Trackbars", A[1][1], 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", A[1][2], 255, nothing)

while True:
    _, frame = cap.read()
    # print(frame.shape)
    # frame = cv2.imread('imageFromFeed.png')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
 
    lower_blue = np.array([l_h, l_s, l_v])
    upper_blue = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # kernel = np.ones((2,2),np.uint8)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
 
    result = cv2.bitwise_and(frame, frame, mask=mask)
 
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("result", result)
 
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
 
cv2.destroyAllWindows()



#this is for orange and dark green
# A=np.array([ [15, 100, 100], [30, 255, 255] ])
# A=np.array([ [60, 130, 0], [132, 255, 255] ])

#this is for pink and light brown
# A=np.array([ [145, 34, 125], [185, 75, 207] ])
# A=np.array([ [14, 110, 107], [26, 255, 255] ])