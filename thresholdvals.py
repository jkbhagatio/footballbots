import numpy as np
import scipy
import scipy.signal as ss
import matplotlib.pyplot as plt
from time import time
import cv2
import pickle
import imutils

with open('arena_ims.pkl', 'rb') as f:
    balldata = pickle.load(f)

with open('blueimg.pkl', 'rb') as g:
    bluedata = pickle.load(g)

with open('goalimg.pkl', 'rb') as h:
    goaldata = pickle.load(h)

lower_rgb_ball = np.array([100,0,50])
upper_rgb_ball = np.array([255,70,250])

lower_hsv_ball = np.array([100,120,100])
upper_hsv_ball = np.array([255,255,200])

lower_rgb_yellow = np.array([180,120,50])
upper_rgb_yellow = np.array([220,160,80])

lower_hsv_yellow = np.array([25,150,170])
upper_hsv_yellow = np.array([35,190,210])

lower_rgb_green = np.array([50,50,30])
upper_rgb_green = np.array([110,120,60])

lower_hsv_green = np.array([35,70,60])
upper_hsv_green = np.array([45,90,80])

lower_rgb_blue = np.array([10,40,80])
upper_rgb_blue = np.array([35,70,170])

lower_hsv_blue = np.array([150,170,120])
upper_hsv_blue = np.array([170,220,170])

upper_hls_ball = np.array([150,85,60])
upper_hls_ball = np.array([180,255,255])

d_e_it = 2

def grayscale(rgb_img):
    return 0.299 *rgb_img[:,:,0] + 0.587 *rgb_img[:,:,1] + 0.114 *rgb_img[:,:,2]

def threshold(frame,rgb_low,rgb_high,name):
    rgbmask = cv2.inRange(frame, rgb_low, rgb_high)
    #cv2.imwrite(name+'mask'+'.jpg',rgbmask)
    return rgbmask

def get_ball(rgb_img, thresh_low,thresh_high,name,detect='edge', preprocess='thresh', plot=False):
    img_to_use = threshold(rgb_img,thresh_low,thresh_high,name) if preprocess == 'thresh' else grayscale(rgb_img)
    height, width = img_to_use.shape
    start = time()

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
    elif detect =='centroid':
        gauss_blurred_frame = (cv2.GaussianBlur(img_to_use, ([5, 5, 1])))
        hsv_lower = (160, 160, 140)
        hsv_upper = (255, 255, 255)
        hsv_frame = cv2.cvtColor(gauss_blurred_frame, cv2.COLOR_RGB2HSV)
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
            best_loc = np.round((x,y)).astype(int)
        if plot:
            plt.imshow(rgb_img)
            plt.imshow(img_to_use, cmap='Blues', alpha=np.where(img_to_use > 0, 0.5, 0))
            plt.plot(best_loc[0],best_loc[1],'*')
            plt.show()
    elif detect == 'sum':
        best_loc = None
        masksumrow = img_to_use.sum(axis = 0)
        masksumcol = img_to_use.sum(axis = 1)
        bestrow = max(masksumrow)
        bestrowindex = np.where(masksumrow == bestrow)
        if np.ptp((np.argsort(masksumrow))[-11:-1]) < 50:
            best_loc = bestrowindex[0][0], np.where(masksumcol == max(masksumcol))
        if plot:
            plt.imshow(rgb_img)
            plt.imshow(img_to_use, cmap='Blues', alpha=np.where(img_to_use > 0, 0.5, 0))
            plt.plot(best_loc[0],best_loc[1],'*')
            plt.show()
    print((time() - start)/1000)
    print(best_loc)
    return best_loc

def masks(datarange,rgb_low,rgb_high,hsv_low,hsv_high,name):
    for i in range(0,len(datarange)):
        frame = datarange[i]

        #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

        rgbmask = cv2.inRange(frame, rgb_low, rgb_high)

        cv2.imwrite(name+'mask'+str(i)+'.jpg',rgbmask)

        frame3 = cv2.bitwise_and(frame, frame, mask=rgbmask)    

        hsv = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
        
        maskhsv = cv2.inRange(hsv, hsv_low, hsv_high)

        cv2.imwrite(name+'hsv'+str(i)+'.jpg',maskhsv)

        hsl = cv2.cvtColor(frame,cv2.COLOR_RGB2HLS)

        cv2.imwrite(name+'hsl'+str(i)+'.jpg',maskhsv)

    return rgbmask, maskhsv

#bluea, blueb = masks(bluedata,lower_rgb_blue,upper_rgb_blue,lower_hsv_blue,upper_hsv_blue,"blue/thresh")
#yella, yellb = masks(goaldata,lower_rgb_yellow,upper_rgb_yellow,lower_hsv_yellow,upper_hsv_yellow,"yellow/thresh")
#greena, greenb = masks(goaldata,lower_rgb_green,upper_rgb_green,lower_hsv_green,upper_hsv_green,"green/thresh")

#reda, redb = masks(bluedata,lower_rgb_ball,upper_rgb_ball,lower_hsv_ball,upper_hsv_ball,"red/hslblue")

def compare(dataset,rgb_below,rgb_behigh,newname):
    for i in range(0,len(dataset)):
        for a in ['edge','contour','centroid','sum']:
            location_edge = get_ball(i, thresh_low=rgb_below,thresh_high=rgb_behigh,name=newname,detect=a, preprocess='thresh', plot=True)

compare(bluedata,lower_rgb_ball,upper_rgb_ball,"test")

            
