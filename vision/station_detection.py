# EENG 437/507
# Nathan Wasniak & Matthew Murray 
# 11/10/2023

import cv2
import os
import numpy as np

images = []
numImages = 19

# Create array of images to open


for i in range(0,19):
    if i < 10:
        str = "0" + i
    else: # i >= 10
        str = i
    images.append(cv2.imread('station'+str+'.jpg'))

for img in images:
    cv2.imshow(img)