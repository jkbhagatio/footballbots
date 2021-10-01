'''
-----
The logic of robot control will be to run `python -m football_commands` in the terminal, which will run
the `main()` function of this file. In the terminal, you will see a prompt to upload `football_response.ino`
to the arduino, and then press a key to continue running 'football_commands'.

ctrl+f 'todo' in each of these two files to find what we still need to do.
-----

Do computer vision to detect ball (pink), opponent (blue), opponent goal (yellow), and our goal (green).
Based on what's detected, send commands to arduino via serial to act (see `football_response.ino`)

Helper functions:
`init`
`behave`
    `get_centroid`
    `send_serial`

Usage examples:
- In terminal, `cd` to directory containing this file and run: `python -m football_commands`.
'''

# Imports
from collections import deque
import time
import argparse
import os
import pathlib
from pathlib import Path
import random
import time

import serial
import utils
import imutils
from imutils.video import VideoStream
import numpy as np
import cv2

import pdb


def main():
    # Parse input args and initialize
    (args, vs, ser) = init()
    # Set color-space ranges:
    col_ranges = {
        'lower_hsv_ball': (160, 160, 140),
        'upper_hsv_ball': (255, 255, 255),
        'lower_rgb_ball': (100, 0, 50),
        'upper_rgb_ball': (255, 70, 200),
        'lower_rgb_yellow': (180, 120, 50),
        'upper_rgb_yellow': (220, 160, 80),
        'lower_rgb_green': (50, 50, 50),
        'upper_rgb_green': (110, 120, 60),
    }
    # Run behavior.
    behave(args, col_ranges, vs, ser)


def init():
    # Parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("--player", default='scorer', help="scorer, goalie, or defender")
    ap.add_argument("--video", default=None, help="path to the (optional) video file")
    ap.add_argument("--frame-size", default=[640, 480], help="frame size of frames in video")
    ap.add_argument("--ball-color", default='pink', help="ball color ('pink' or 'cyan')")
    ap.add_argument("--gaussian-filter-params", default=[5, 5, 1], help="3 numbers: filter kernel width, height, and sigma (default: [5, 5, 1])")
    ap.add_argument("--min-ball-radius", default=10, help="min pixels for detecting ball")
    ap.add_argument("--min-goal-radius", default=10, help="min pixels for detecting our or opponent's goal")
    ap.add_argument("--min-bot-radius", default=10, help="min pixels for detecting our or opponent's goal")
    ap.add_argument("--move-thresh", default=30, help="pixels threshold for determining movement")
    ap.add_argument("--sight-thresh", default=300, help="pixels threshold for determining movement")
    ap.add_argument("--max-turn-time", default=5, help="max time (in s) for turning to look for ball, before moving forward")
    ap.add_argument("--forward-search-time", default=2, help="max time (in s) for moving forward to look for ball before turning")

    args = vars(ap.parse_known_args()[0])
    
    # Initialize serial.
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
    input("Run arduino code, then press any key to continue")
    
    # Initialize video.
    # If vid file not specified, get webstream, else get vid file.
    if args['video'] is None:
        vs = VideoStream(resolution=(640, 480), framerate=20, usePiCamera=True, src=0).start()
    else:
        vs = cv2.VideoCapture(args['video'])

    return args, vs, ser


def behave(args, col_ranges, vs, ser):
    # Perform tracking and action commands to arduino frame-by-frame.
    # Set vals from args.
    PLAYER = args['player']
    FRAME_CENTER = (args['frame_size'][0] / 2, args['frame_size'][1] / 2)
    MOVE_THRESH = args['move_thresh']
    SIGHT_THRESH = args['sight_thresh']
    GAUSS_FILT_PARAMS = args['gaussian_filter_params']
    MIN_PX_BALL_RAD = args['min_ball_radius']
    D_E_IT = 2;  # dilation/erosion iterations
    last_cmd = None
    turn_timer = 0
    # While we haven't terminated the video stream, or while video file still has frames...:
    pdb.set_trace()
    while True:
        frame = vs.read()  # grab the current frame
        frame = frame[1] if not (args['video'] is None) else frame  # handle frame from video file or stream
        # Break on empty frame or 'q' key
        if frame is None:
            break
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
        # todo: define precise behavior for all player types
        # todo: functions for `find_ball()`, `has_ball()`, `find_oppo_goal()`, `see_oppo_player()`
        if PLAYER == 'scorer':
            # Find ball
            # Get frame centroid
            #(todo: if want to test other detection methods, replace 'get_centroid' with that function)
            masked_frame = mask_frame(frame, GAUSS_FILT_PARAMS, col_ranges['lower_rgb_ball'],
                                      col_ranges['upper_rgb_ball'])
            # todo: replace None with 'min_area'
            pdb.set_trace()
            (centroid, radius) = get_centroid(masked_frame, D_E_IT, None)
            see_ball = True if centroid is not None else False
            if see_ball:
                last_cmd = move_to_obj(centroid, frame_center, move_thresh, sight_thresh, ser, cmd, last_cmd)
            else:
                #explore():
                # If ball not in frame:
                # If (cur_time - turn_timer) > max_turn_time:
                # move forward for 'forward_search_time'
                # Start turning, and compare current time to 'turn+timer.
                # If this time exceeds 'max_turn_time', move forward for 'forward_search_time', before turning again.
                pass
            # if has_ball: find_oppo_goal
            # if has_ball and sees oppo goal, blow fan and move forward.
            # if has_ball and doesn't see oppo goal, rotate/move until see oppo goal.
            
            
            # Send to serial:
            # If (x - frame_center) < abs(move threshold): move forward;
            # elif (x - frame_center) > abs(move_threshold) but < abs(sight_threshold): move in appropriate direction
            # elif (x - frame_center) > abs(sight_threshold), turn in place within time threshold
        elif PLAYER == 'defender':
            # find_ball()
            # if sees_ball and doesn't see own goal, blow ball
            # if sees oppo_bot, attack?
            pass
        elif PLAYER == 'goalie':
            # move near goalline and blow ball away from goal? ensure min movement requirement is satisfied
            pass
    
    # End behavior: stop the stream or release the camera:
    if args['video'] is None:
        vs.stop()
    else:
        vs.release()
    cv2.destroyAllWindows()

    
# Tracking on single image: looks for centroid of `obj` (a ball, goal, or oppo bot)
# Steps: 1) blur; 2) convert to color space; 3) mask based on
# upper and lower color space bounds; 4) dilate; 5) erode; 6) find contours;
# 7) find largest contour; 8) compute min enclosing circle and its center
# todo: if...else on 'obj' (ball, oppo player, our goal, oppo goal)
# todo: define min_area the different obj
def mask_frame(frame, gauss_filt_params, lower, upper):
    gauss_blurred_frame = (
            cv2.GaussianBlur(frame, (gauss_filt_params[0], gauss_filt_params[1]), gauss_filt_params[2]))
    col_space_frame = cv2.cvtColor(gauss_blurred_frame, cv2.COLOR_BGR2RGB)
    masked_frame = cv2.inRange(col_space_frame, lower, upper)
    return masked_frame

def get_centroid(masked_frame, d_e_it, min_area):
    dilated_frame = cv2.dilate(masked_frame, None, iterations=d_e_it)
    eroded_frame = cv2.erode(dilated_frame, None, iterations=d_e_it)
    contours = cv2.findContours(eroded_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None  # initialize as object not found
    radius = 0
    # Only proceed with tracking if contours found in this frame
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        center = np.round((x,y)).astype(int)

    return center, radius


def see_obj():
    pass


def move_to_obj(centroid, frame_center, move_thresh, sight_thresh, ser, cmd, last_cmd):
    if abs(centroid[0] - FRAME_CENTER[0]) < MOVE_THRESH:  # go forward
        last_cmd = send_serial('f', last_cmd, ser)
    elif ((abs(centroid[0] - FRAME_CENTER[0]) > MOVE_THRESH)
        and (abs(centroid[0] - FRAME_CENTER[0]) < SIGHT_THRESH)):
        if centroid[0] < FRAME_CENTER[0]:  # go right-forward
            last_cmd = send_serial('r', last_cmd, ser)
        else:  # go left-forward
            last_cmd = send_serial('l', last_cmd, ser)
    return last_cmd
#     else:  # turn in place
#         c = random.choice(['a', 'd'])  # 'a' for turn left, 'd' for turn right (equivalent to arrow keys)
#         last_cmd = send_serial(c, last_cmd, ser)

# Sends a command to serial if the command is different from the previous command sent
def send_serial(c, last_cmd, ser):
    if last_cmd != c:
        ser.write(c.encode('utf-8'))
        last_cmd = c
    return last_cmd


if __name__ == '__main__':
    main()
