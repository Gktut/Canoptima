import cv2
import numpy as np
import serial
from time import sleep
import threading
from statistics import mode

h = 240
w = 320

frame1 = np.zeros((w,h,3))
frame2 = np.zeros((w,h,3))

#DD 1=fw north 2=bw south 3=right east 4=left west DD
reset = 0
loop_check = 0
move_fb = 0
move_rl = 0
count_fb = 0
count_rl = 0
move_fb_array = [0,0,0,0,0,0,0,0,0,0]
move_rl_array = [0,0,0,0,0,0,0,0,0,0]

def optimize():
    global count_rl
    global move_rl_array
    global count_fb
    global move_fb_array
    global loop_check
    global reset
    command = bytearray()
    if(loop_check > 699):
        ## (1) Crop the bounding rect
        pts1 = np.array([[0,240],[0,175],[115,140],[320,200],[320,240]])
        rect1 = cv2.boundingRect(pts1)

        pts2 = np.array([[0,240],[0,175],[115,140],[320,200],[320,240]])
        rect2 = cv2.boundingRect(pts2)
        x1,y1,w1,h1 = rect1
        x2,y2,w2,h2 = rect2
        croped1 = frame1[y1:y1+h, x1:x1+w].copy()
        croped2 = frame2[y2:y2+h, x2:x2+w].copy()
        ## (2) make mask
        pts1 = pts1 - pts1.min(axis=0)
        pts2 = pts2 - pts2.min(axis=0)
        
        mask1 = np.zeros(croped1.shape[:2], np.uint8)
        cv2.drawContours(mask1, [pts1], -1, (255, 255, 255), -1, cv2.LINE_AA)
 
        mask2 = np.zeros(croped2.shape[:2], np.uint8)
        cv2.drawContours(mask2, [pts2], -1, (255, 255, 255), -1, cv2.LINE_AA)
        ## (3) do bit-op
        dst1 = cv2.bitwise_and(croped1, croped1, mask=mask1)
        dst2 = cv2.bitwise_and(croped2, croped2, mask=mask2)
        ## (4) add the white background
        bg1 = np.ones_like(croped1, np.uint8)*255
        cv2.bitwise_not(bg1,bg1, mask=mask1)
        bg2 = np.ones_like(croped2, np.uint8)*255
        cv2.bitwise_not(bg2,bg2, mask=mask2)
        
        dst11 = bg1+ dst1
        dst22 = bg2+ dst2
#        print('Frame1: ',np.mean(dst1)+np.mean(dst11),'\n')
#        print('Frame2: ',np.mean(dst2)+np.mean(dst22),'\n')
        if((np.mean(dst1)+np.mean(dst11))>320 or (np.mean(dst2)+np.mean(dst22))>320):
            loop_check = 0
            reset = 1
    loop_check = loop_check+1
    command.append(0xDD)
    if(reset == 0):
        if(move_fb==1):
            command.append(0x01)
        elif(move_fb==2):
            command.append(0x02)
        else:
            command.append(0x00)
            
        if(move_rl==1):
            command.append(0x01)
        elif(move_rl==2):
            command.append(0x02)
        else:
            command.append(0x00)
        command.append(0xDD)
        threading.Timer(0.04,optimize).start()
    else:
        command.append(0x03)
        command.append(0x03)
        command.append(0xDD)
        reset = 0
        threading.Timer(0.04,optimize).start()
    ser.write(command)


#serial connection establish
ser = serial.Serial("/dev/ttyS0",9600)

threading.Timer(0.04,optimize).start()

# Load the video capture object

#image acquisition
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cap1 = cv2.VideoCapture(2)
cap2 = cv2.VideoCapture(0)

# Set the camera resolution
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

a = 0
b = 0
x0 = 0
y0 = 0
x1 = 0
y1 = 0
x2 = 0
y2 = 0

# Loop until the user presses the "q" key
while True:
    # Capture the current frame
    _, frame1 = cap1.read()
    _, frame2 = cap2.read()
    
    if frame1 is None:
        print("hata1")
        continue
    if frame2 is None:
        print("hata2")
        continue
    frame1 = cv2.resize(frame1,(w,h))
    frame2 = cv2.resize(frame2,(w,h))

    # The HSV values for thresholding

    hMin = 0
    sMin = 0
    vMin = 150
    hMax = 177 
    sMax = 255
    vMax = 175
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Apply thresholding to the image HSV and threshold
    hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv1, lower, upper)
    thresholded_img1 = cv2.bitwise_and(frame1, frame1, mask=mask1)

    hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
    mask2 = cv2.inRange(hsv2, lower, upper)
    thresholded_img2 = cv2.bitwise_and(frame2, frame2, mask=mask2)
   
    # Perform edge detection using Canny edge detector
    min_threshold = 100
    max_threshold = 150
    edge_img1 = cv2.Canny(thresholded_img1, min_threshold, max_threshold)
    edge_img2 = cv2.Canny(thresholded_img2, min_threshold, max_threshold)

    # Apply Hough transform to detect lines in the edge image
    rho_resolution = 1
    theta_resolution = np.pi / 180
    threshold = 60
    lines1 = cv2.HoughLines(edge_img1, rho_resolution, theta_resolution, threshold)
    lines2 = cv2.HoughLines(edge_img2, rho_resolution, theta_resolution, threshold)
    
    # If no lines detected to prevent errors in the rest lines are defined
    if lines1 is None:
        lines1 = []
    if lines2 is None:
        lines2=[]
    
    # The lines that are perpendicular to each other for each frame is defined  
    f1_line_perp1 = []
    f1_line_perp2 = []

    f2_line_perp1 = []
    f2_line_perp2 = []

    # The color codes and line features are defined
    line_color = (255, 0, 0)
    corner_color = (0, 0, 255)
    line_thickness = 2
    corner_radius = 5
    gray = np.zeros((480,640,3))

    # For frame 1 lines are distinguished as perp1 and perp2 as they are perpendicular to each other.
    # And we use the slope of lines to seperate the lines that are useful.

    for line in lines1:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        if (x2-x1) != 0:
            slope = ((y2-y1) / (x2-x1))
        else:
            slope = 1000
            
        if (0.20<slope<0.60):
            f1_line_perp1.append(line)
        elif (-0.60<slope<-0.20):
            f1_line_perp2.append(line)
            
    # We draw 2 lines that are detected on the first camera in the desired slope

    if f1_line_perp1:
        rho1, theta1 = f1_line_perp1[0][0]
        a = np.cos(theta1)
        b = np.sin(theta1)
        x0 = a * rho1
        y0 = b * rho1
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(frame1, (x1, y1), (x2, y2), line_color, line_thickness)
        
    cv2.line(frame1,(115,140),(320,200),(0,255,0),3)
    # if true then the left edge of the shadow is inside and the canopy needs to move left
    move_left = ((((y1+y2)/2)-170) > (((x1+x2)/2)-220)*(0.293))
#    print("move left : ",move_left,"\n")
        
    if f1_line_perp2:
        rho2, theta2 = f1_line_perp2[0][0]
        a = np.cos(theta2)
        b = np.sin(theta2)
        x0 = a * rho2
        y0 = b * rho2
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(frame1, (x1, y1), (x2, y2), line_color, line_thickness)
        
    cv2.line(frame1,(115,140),(0,175),(0,255,0),3)
    # if true then the back edge of the shadow is inside and the canopy needs to move backwards
    move_backwards = ((((y1+y2)/2)-157.5) > (((x1+x2)/2)-57.5)*(-0.304))
   # print("move backwards : ",move_backwards,"\n")

    # For frame 2 lines are distinguished as perp1 and perp2 as they are perpendicular to each other.
    # And we use the slope of lines to seperate the lines that are useful.

    for line in lines2:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        if (x2-x1) != 0:
            slope = ((y2-y1) / (x2-x1))
        else:
            slope = 1000
            
        if (0.25<slope<0.45):
            f2_line_perp1.append(line)
        elif (-0.70<slope<-0.20):
            f2_line_perp2.append(line)
            
    # We draw 2 lines that are detected on the first camera in the desired slope

    if f2_line_perp1:
        rho1, theta1 = f2_line_perp1[0][0]
        a = np.cos(theta1)
        b = np.sin(theta1)
        x0 = a * rho1
        y0 = b * rho1
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(frame2, (x1, y1), (x2, y2), line_color, line_thickness)
        
    cv2.line(frame2,(115,140),(320,200),(0,255,0),3)
    # if true then the right edge of the shadow is inside and the canopy needs to move right
    move_right = ((((y1+y2)/2)-170) > (((x1+x2)/2)-220)*(0.293))
#    print("move right : ",move_right,"\n")
    
    if f2_line_perp2:
        rho2, theta2 = f2_line_perp2[0][0]
        a = np.cos(theta2)
        b = np.sin(theta2)
        x0 = a * rho2
        y0 = b * rho2
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(frame2, (x1, y1), (x2, y2), line_color, line_thickness)
        
    cv2.line(frame2,(115,140),(0,175),(0,255,0),3)
    # if true then the forward edge of the shadow is inside and the canopy needs to move forward
    move_forwards = ((((y1+y2)/2)-157.5) > (((x1+x2)/2)-57.5)*(-0.304))
#    print("move forwards : ",move_forwards,"\n")

    if (move_forwards==True):
        move_fb = 1
        if(mode(move_fb_array) == 2):
            move_fb_array[count_fb] = move_fb
            count_fb = (count_fb+1)%10
            move_fb = 0
    elif (move_backwards==True):
        move_fb = 2
        if(mode(move_fb_array) == 1):
            move_fb_array[count_fb] = move_fb
            count_fb = (count_fb+1)%10
            move_fb = 0
    else:
        move_fb = 0
    
    if (move_right==True):
        move_rl = 1
        if(mode(move_rl_array) == 2):
            move_rl_array[count_rl] = move_rl
            count_rl = (count_rl+1)%10
            move_rl = 0
    elif (move_left==True):
        move_rl = 2
        if(mode(move_rl_array) == 1):
            move_rl_array[count_rl] = move_rl
            count_rl = (count_rl+1)%10
            move_rl = 0
    else:
        move_rl = 0

    cv2.imshow('Image-1 with lines and corner', frame1)
    cv2.imshow('Image-2 with lines and corner', frame2)
    #cv2.imshow('HSV1',thresholded_img1)
    #cv2.imshow('HSV2',thresholded_img2)

# if the user presses 'q', break the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
