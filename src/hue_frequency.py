import skimage

import cv2
import imutils
import os
import matplotlib.pyplot as plt
import numpy as np

# import warnings
# warnings.simplefilter(action='ignore', category=FutureWarning)
# warnings.simplefilter(action='ignore', category=UserWarning)


DIRECTORY = './mall'
keyword = 'actual'
# DIRECTORY = '~/.ros'
for im1 in os.listdir(DIRECTORY):
# for img in os.listdir('~/.ros'):
    # img = cv2.imread("~/.ros/1_actual.jpg")
    if keyword in im1:
        img = (skimage.io.imread(os.path.join(DIRECTORY,im1)))
        new_height, new_width, channels = img.shape
        new_height = int(900)
        new_width = int(900)

        img = imutils.resize(img, height=new_height, width=new_width)
        new_height, new_width, channels = img.shape

        output = img
        output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

        #HUE HISTOGRAM
        x = output[:,:,0].flatten()
        plt.hist(x, bins=18)
        plt.ylabel('Frequency')
        plt.xlabel('Hue(mapped 0-180)')
        plt.show()
        break

