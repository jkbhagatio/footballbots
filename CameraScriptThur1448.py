import numpy as np
import cv2
import serial
import time
import PIL
from PIL import Image
from collections import deque
import time
import argparse
import imutils
import random

ser = serial.Serial()
ser.baudrate = 19200
ser.port = '/dev/ttyUSB0'
ser.open()

def smooth(x,window_len=11,window='hanning'):
    s=np.r_[x[window_len-1:0:-1],x,x[-2:-window_len-1:-1]]
    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y=np.convolve(w/w.sum(),s,mode='valid')
    return y

def return_coors(frame, red_T=150, green_T=100):
    imagearrayr = frame[0]
    imagearrayg = frame[1]
    wherehthresh = np.where((imagearrayg < red_T)&(imagearrayr > green_T), 255, 0)
    collapsed_columns = smooth(np.squeeze(np.sum(wherehthresh, axis=0)))
    collapsed_rows = smooth(np.squeeze(np.sum(wherehthresh, axis=1)))
    row_pos = int(np.mean(np.where(collapsed_rows == np.max(collapsed_rows))[0]))
    col_pos = int(np.mean(np.where(collapsed_columns == np.max(collapsed_columns))[0]))
    return row_pos, col_pos

def on_left_go_right():
    ser.write(b'x')
    right_switch = False

def on_right_go_left():
    ser.write(b'o')
    left_swtich = False
    
def stop():
    ser.write(b'v')
    right_switch = True
    left_switch = True
    
def forward():
    ser.write(b'w')
    right_switch = True
    left_switch = True

def randoms():
    r = random.randint(0,1)
    r_time = random.randint(0,100)
    if r == 0:
        on_right_go_left()
        time.sleep(r_time/100)
        ret, frame = cap.read()
        frame = np.array(frame)
        contours, checker = get_contours(frame)
        if checker == 0:
            randoms()
    elif r == 1:
        on_left_go_right()
        time.sleep(r_time/100)
        ret, frame = cap.read()
        frame = np.array(frame)
        contours, checker = get_contours(frame)
        if checker == 0:
            randoms()

def update_image():
    for i in range(imagearray.shape[1]):
        imagearray[0][i,y] = 0
    for i in range(imagearray.shape[0]):
        imagearray[0][x,i] = 0

def get_contours(frame):
    eroded_frame = cv2.erode(cv2.dilate(cv2.inRange(cv2.cvtColor(cv2.GaussianBlur(frame, GAUSS_FILT_PX_SZ, GAUSS_FILT_SIGMA), cv2.COLOR_BGR2HSV), hsv_lower, hsv_upper), None, iterations=D_E_IT), None, iterations=D_E_IT)
    contours = cv2.findContours(eroded_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None
    checker = len(contours)
    return contours, checker

def get_xy(contours):
    c = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    center = np.round((x,y)).astype(int)
    x, y = center[0], center[1]
    return x, y 
    

cap = cv2.VideoCapture(0)

left_switch = True
right_switch = True
x_thresh = 150
counter = 0
MIN_PX_BALL_RAD = 10
GAUSS_FILT_PX_SZ = (5, 5)
GAUSS_FILT_SIGMA = 1
D_E_IT = 2  
hsv_lower = (160, 160, 140)
hsv_upper = (255, 255, 255)

def main():
    while(True):
        ret, frame = cap.read()
        frame = np.array(frame)
        contours, checker = get_contours(frame)
        list_locals = locals().keys()
        if checker == 0:
            if 'x' not in list_locals:
                main()
            else:
                randoms()
        else:
            x, y = get_xy(contours)
            mid_x = frame.shape[1]/2
            lower_bound = mid_x-x_thresh
            upper_bound = mid_x+x_thresh
            if lower_bound < x < upper_bound:
                forward()
            elif x < lower_bound:
                if right_switch == True:
                    on_left_go_right()
            elif x > upper_bound:
                if left_switch == True:
                    on_right_go_left()
    
main()
