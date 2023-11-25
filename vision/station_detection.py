# EENG 437/507
# Nathan Wasniak & Matthew Murray 
# 11/24/2023

import cv2
import matplotlib.pyplot as plt
import numpy as np
import copy

images = []
# RGB_images = []
numImages = 13

# Create array of images to open
for i in range(1,numImages):
    word = ""
    if i < 10:
        word = "0" + str(i)
    else: # i >= 10
        word = str(i)
    # print('./vision/dockingStationImages/station'+word+'.jpg')
    img = cv2.imread('./vision/dockingStationImages/station'+word+'.jpg')
    
    # Convert BGR to RGB (OpenCV uses BGR by default)
    images.append(img)
    # RGB_images.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    # cv2.imshow(word,img)
# cv2.waitKey(0)

def subplot_images(rows = 3, columns = 4, images = images):
    fig, axes = plt.subplots(nrows=rows, ncols=columns, figsize=(columns*3, rows*3))
    for num in range(1, rows*columns+1):
        fig.add_subplot(rows, columns, num)
        
        idx = num - 1
        
        plt.imshow(images[idx], aspect='auto')
        
    fig.tight_layout() # used to adjust padding between subplots 
    plt.show()

# Make and create new array of images in grayscale
gray_images = []
for i in range(0, numImages-1):
    img = images[i]
    gray_images.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))



img1 = images[0]
img2 = gray_images[0]
orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# Match descriptors.
matches = bf.match(des1,des2)
# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)
# Draw first 10 matches.
img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.imshow(img3),plt.show()

# apply thresholding to convert grayscale to binary image
ret,binary_image = cv2.threshold(gray_images[0],130,255,0)

# Display the Binary Image
imS = cv2.resize(binary_image, (960, 540))  # Resize image to fit on screen
cv2.imshow("Binary Image", imS)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Load in template images to match and see current orientation of charging station
behind_station = cv2.imread('./vision/stationTemplates/behindStation.jpg')
front_station = cv2.imread('./vision/stationTemplates/frontStation.jpg')
left_front_station = cv2.imread('./vision/stationTemplates/leftFrontStation.jpg')
left_station = cv2.imread('./vision/stationTemplates/leftSideStation.jpg')
right_station = cv2.imread('./vision/stationTemplates/rightSideStation.jpg')
templates = [behind_station, front_station, left_front_station, left_station, right_station]

best_matches = []
for img2 in templates:
    img1 = images[0]

    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
    best_matches.append(matches)
    # Draw first 10 matches.
    img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img3),plt.show()

smallest = 100e6
index = -1
# compare which template has the best match
for i in range(0,len(best_matches)):
    matches = best_matches[i]
    matches = matches[:10] # Cut off all the matches except for the best 10
    avg=0
    for j in matches:
        avg += j.distance
    avg = avg / len(matches)
    print("avg:")
    print(avg)
    print("Distance:")
    print(matches[0].distance)
    dist = matches[0].distance
    if dist < smallest:
        smallest = dist
        index = i

print("Best Match:")
print(index)
# print(matches[:10])
