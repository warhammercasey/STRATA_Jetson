# EENG 437/507
# Nathan Wasniak & Matthew Murray 
# 11/10/2023

import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

images = []
numImages = 19

# Create array of images to open
for i in range(0,19):
    if i < 10:
        str = "0" + i
    else: # i >= 10
        str = i
    img = cv2.imread('./vision/station'+str+'.jpg')
    # Convert BGR to RGB (OpenCV uses BGR by default)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    images.append(img)

fig, axes = plt.subplots(1, 3, figsize=(12, 4))

for img in images:
    cv2.imshow(img)
    