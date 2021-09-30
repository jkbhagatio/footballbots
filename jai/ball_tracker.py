'''
Do ball tracking from an image or webcam stream.

Input args for: 1) video source, 2) ball color, 3) writing out processed stream to a file,
4) writing out processed stream to a web server via flask, 5) contrail buffer for writing
out processed stream, 6) gaussian filter params, 7) minimum ball radius for viz

Functions for: 1) finding centroid, 2) displaying tracking

Usage examples:

# Track a pink ball from a video stream without plotting the image:

# Track a pink ball from a video file with default contrail buffer size, write out file.
python -m ball_tracker --video /home/pi/swc_bootcamp_2021/media/ben_pink_ball_move.h264 --ball-color pink --send-serial True 

'''

# Imports
from collections import deque
import time
import argparse
import os
import pathlib
from pathlib import Path

import serial
import utils
import imutils
from imutils.video import VideoStream
import numpy as np
import cv2
import pdb

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=None, help="path to the (optional) video file")
    ap.add_argument("--frame-size", default=[640, 480], help="frame size of frames in video")
    ap.add_argument("--ball-color", default='pink', help="ball color ('pink' or 'cyan')")
    ap.add_argument("--write-file", default=None, help="path to name of video file to write to")
    ap.add_argument("--write-stream", default=False, help="writes to web server stream if True")
    ap.add_argument("--deque-buffer-size", type=int, default=16, help="max buffer size")
    ap.add_argument("--gaussian-filter-params", default=[5, 5, 1], help="3 numbers: filter kernel width, height, and sigma (default: [5, 5, 1])")
    ap.add_argument("--min-ball-radius", default=10, help="pixels for ball radius for visualization of tracking")
    ap.add_argument("--send-serial", default=False, help="send position of ball relative to center to serial")
    ap.add_argument("--move-thresh", default=30, help="pixels threshold for determining movement")
    ap.add_argument("--sight-thresh", default=300, help="pixels threshold for determining movement")

    args = vars(ap.parse_known_args()[0])
    
    # Initialize serial.
    if args['send_serial']:
        ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        last_command = '0'  # last command placeholder
        input("Run arduino code, then press any key to continue")

    # Set vals from input args
    FRAME_CENTER = (args['frame_size'][0] / 2, args['frame_size'][1] / 2)
    MOVE_THRESH = args['move_thresh']
    SIGHT_THRESH = args['sight_thresh']
    GAUSS_FILT_PARAMS = args['gaussian_filter_params']
    MIN_PX_BALL_RAD = args['min_ball_radius']
    D_E_IT = 2;  # dilation/erosion iterations

    # Initialize settings before tracking
    # If vid not specified, get webstream, else get vid.
    if args['video'] is None:
        vs = VideoStream(src=0).start()
    else:
        vs = cv2.VideoCapture(args['video'])
    
    # HSV ranges of ball colors found using range detector tool: `imutils_range_detector`
    if args['ball_color'] == 'cyan':
        hsv_lower = (95, 80, 115)
    elif args['ball_color'] == 'pink':
        hsv_lower = (160, 160, 140)
    else:
        raise ValueError
            #print(f'{args['ball-color']} is not a valid color')
    hsv_upper = (255, 255, 255)

    # Set up video writer if specified
    #if ((args['write_file'] is not None) or args['write_stream']):
        #fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        #writer = cv2.VideoWriter('/home/pi/swc_bootcamp_2021/media/ben_pink_ball_tracked.mp4',
                                  #fourcc, fps=20.0, frameSize=(640,  480), isColor=True)

    # Perform tracking frame-by-frame while we haven't terminated video stream,
    # or while video file still has frames
    pdb.set_trace()
    while True:
        # grab the current frame
        frame = vs.read()
        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if not (args['video'] is None) else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break
        # Get frame centroid
        (centroid, radius) = get_centroid(frame, GAUSS_FILT_PARAMS, hsv_lower, hsv_upper, D_E_IT)
        # Draw frame and write out file / stream if specified.
        #if ((args['write_file'] is not None) or args['write_stream']):
            #draw_frame(center, radius)
            #write to file or stream
        # Send to serial
        if args['send_serial']:
            # If (x - frame_center) < abs(move threshold): move forward;
            # elif (x - frame_center) > abs(move_threshold) but < abs(sight_threshold): move in appropriate direction
            # elif (x - frame_center) > abs(sight_threshold), turn in place to the left
            # within threshold.
            if abs(centroid[0] - FRAME_CENTER[0]) < MOVE_THRESH:
                if last_command != 'f':
                    ser.write('f'.encode('utf-8'))
                    last_command = 'f'
            elif ((abs(centroid[0] - FRAME_CENTER[0]) > MOVE_THRESH)
                   and (abs(centroid[0] - FRAME_CENTER[0]) < SIGHT_THRESH)):
                if centroid[0] < FRAME_CENTER[0]:
                    if last_command != 'l':
                        ser.write('l'.encode('utf-8'))
                        last_command = 'l'
                else:
                    if last_command != 'r':
                        ser.write('r'.encode('utf-8'))
                        last_command = 'r'
            else:  # turn in place
                if last_command != 't':
                    ser.write('t'.encode('utf-8'))
                    last_command = 't'
    # Stop the stream or release the camera:
    if args['video'] is None:
        vs.stop()
    else:
        vs.release()
    cv2.destroyAllWindows()


# Tracking on single image:
# Steps: 0) resize; 1) blur; 2) convert to HSV-space; 3) mask based on
# upper and lower hsv bounds; 4) dilate; 5) erode; 6) find contours;
# 7) find largest contour; 8) compute min enclosing circle and its
# center
def get_centroid(frame, gauss_filt_params, hsv_lower, hsv_upper, d_e_it):
    gauss_blurred_frame = (
        cv2.GaussianBlur(frame, (gauss_filt_params[0], gauss_filt_params[1]), gauss_filt_params[2]))
    hsv_frame = cv2.cvtColor(gauss_blurred_frame, cv2.COLOR_BGR2HSV)
    masked_frame = cv2.inRange(hsv_frame, hsv_lower, hsv_upper)
    dilated_frame = cv2.dilate(masked_frame, None, iterations=d_e_it)
    eroded_frame = cv2.erode(dilated_frame, None, iterations=d_e_it)
    contours = cv2.findContours(eroded_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = (10000, 10000)  # initialize high (ball not found)
    radius = 0
    # Only proceed with tracking if contours found in this frame
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        center = np.round((x,y)).astype(int)

    return center, radius


def draw_frame(frame, centroid, radius):
    pass


if __name__ == '__main__':
    main()
