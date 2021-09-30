# import matplotlib
import numpy as np
import scipy
import scipy.signal as ss
import serial
import matplotlib.pyplot as plt
import pickle as pkl
from time import time

import cv2


def grayscale(rgb_img):
    return 0.299 *rgb_img[:,:,0] + 0.587 *rgb_img[:,:,1] + 0.114 *rgb_img[:,:,2]

def threshold(rgb_img):
    return (rgb_img[:, :, 0] > 150) * (rgb_img[:, :, 1] < 70)

def get_ball(rgb_img, detect='edge', preprocess='thresh', plot=False):
    img_to_use = threshold(rgb_img) if preprocess == 'thresh' else grayscale(rgb_img)
    height, width = img_to_use.shape

    if detect == 'edge':
        edge_map = ss.convolve2d(img_to_use, np.array([[-1-1j, -1j, 1-1j], [-1, 0, 1], [-1+1j, 1j, 1+1j]]))[1:-1, 1:-1]
        edge_mask = np.abs(edge_map) > 2.3 if preprocess == 'thresh' else 35
        edge_inds = np.where(edge_mask)
        cos = np.cos(np.angle(edge_map)[edge_mask])
        sin = np.sin(np.angle(edge_map)[edge_mask])
        x_for_div = np.where(cos > 0, edge_inds[1], width-edge_inds[1])
        y_for_div = np.where(sin > 0, edge_inds[0], height-edge_inds[0])
        multipliers = -np.minimum(x_for_div/np.where(np.isclose(cos,0), np.inf, np.abs(cos)), y_for_div/np.where(np.isclose(sin,0), np.inf, np.abs(sin))).astype(int)[:,None]+np.arange(800, dtype=int)[None,:]

        x_inds = (cos[:, None]*multipliers + edge_inds[1][:, None]).astype(int)
        y_inds = (sin[:, None]*multipliers + edge_inds[0][:, None]).astype(int)
        valid_inds = np.where((x_inds >= 0) * (x_inds < width) * (y_inds >= 0) * (y_inds < height) > 0)
        valid_x_inds = x_inds[valid_inds]
        valid_y_inds = y_inds[valid_inds]

        votes = np.bincount(np.append(valid_y_inds*width+valid_x_inds, [height*width-1])).reshape((height,width))
        best_loc = np.argmax(votes)
        print(np.max(votes))
        if votes[best_loc//640, best_loc%640] > 5:
            best_loc = (best_loc//640, best_loc%640)
        else:
            best_loc = None

        if plot:
            plt.imshow(rgb_img)
            plt.imshow(img_to_use, cmap='Blues', alpha=np.where(img_to_use > 0, 0.5, 0))
            plt.imshow(edge_mask, cmap='binary', alpha=np.where(edge_mask > 0, 1.0, 0))
            # plt.imshow(edge_mask, 'Blues')
            print(votes.dtype)
            plt.imshow(votes.astype(float), cmap='Reds', alpha=np.where(votes > 5, 0.5, 0))
            plt.plot([best_loc[1]], [best_loc[0]], '*')
            plt.show()
    elif detect == 'contour-sk':
        raise NotImplementedError
    elif detect == 'contour':
        masked = img_to_use.astype(np.uint8)
        best_loc = None
        if np.any(masked):
            contours,hierarchy = cv2.findContours(masked, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            biggest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(biggest_contour)
            #print(M['m00'])
            if plot:
                plt.imshow(rgb_img)
                plt.imshow(img_to_use, cmap='Blues', alpha=np.where(img_to_use > 0, 0.5, 0))
                for a in contours:
                    plt.plot(a[:,0,0],a[:,0,1], 'b-')
                plt.plot(biggest_contour[:,0,0],biggest_contour[:,0,1], 'r-')
                if M['m00'] > 1000:
                    plt.plot([M['m10']/M['m00']], [M['m01']/M['m00']], '*')
                plt.show()
            if M['m00'] > 1000:
                best_loc = (M['m01']/M['m00'], M['m10']/M['m00'])
    return best_loc
            


# with open('test_ims2.pkl','rb') as f:
#     images = pkl.load(f)
    
# # temp = np.array(images[:,:,:,0])
# # images[:,:,:,0] = images[:,:,:,2]
# # images[:,:,:,2] = temp
# images[:,:,:,0], images[:,:,:,2] = images[:,:,:,2], images[:,:,:,0]

# dm = 'contour'
# for i in range(len(images)):
#     print(get_ball(images[i], plot=True, detect=dm))
# print(get_ball(images[-2], plot=True, detect=dm))
# print(get_ball(images[-1], plot=True, detect=dm))


# start = time()
# for i in range(100):
#   get_ball(images[i % len(images)], detect='contour')
# print((time() - start)/1000)


# # Get video capture object for camera 0
cap = cv2.VideoCapture(0)

# Create named window for diaply
# cv2.namedWindow('preview')

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

# Loop until 'q' pressed
while(True):
    # Read most recent frame
    ret, frame = cap.read()

    loc = get_ball(frame, detect='edge')
    print(loc)
    if loc is None:
        ser.write('B\n'.encode('UTF-8'))
        # if np.random.random() < 0.3:
        #     ser.write('F\n'.encode('UTF-8'))
        # else:
        #     ser.write('L\n'.encode('UTF-8'))
    else:
        if loc[1] < 360 and loc[1] > 280:
            ser.write('F\n'.encode('UTF-8'))
        elif loc[1] > 360:
            ser.write('R\n'.encode('UTF-8'))
        else:
            ser.write('L\n'.encode('UTF-8'))

    # Convert to grayscale
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Computer Vision Code
    # --------------------
    # thresh = 128
    # ret, binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    # --------------------

    # Display the resulting frame
    # cv2.imshow('preview', binary)

    # Wait for a keypress, and quit if 'q'
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Release the caputre
cap.release()

# Destroy display window
# cv2.destroyAllWindows()
