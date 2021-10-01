import pickle
import os
import cv2
import matplotlib.pyplot as plt

img_list = []
outfile = open('goalimg.pkl','wb')

for a in os.listdir('/home/kira/Documents/LastBlackBox/boxes/intelligence/code/goalimages'):
    print(a)
    img_list.append(plt.imread('/home/kira/Documents/LastBlackBox/boxes/intelligence/code/goalimages/'+str(a)))

print(img_list)

pickle.dump(img_list,outfile)   