# EENG 437/507
# Nathan Wasniak & Matthew Murray 
# 11/10/2023

import cv2
import matplotlib.pyplot as plt
import copy

images = []
RGB_images = []
numImages = 13

# Create array of images to open
for i in range(1,numImages):
    word = ""
    if i < 10:
        word = "0" + str(i)
    else: # i >= 10
        word = str(i)
    print('./vision/dockingStationImages/station'+word+'.jpg')
    img = cv2.imread('./vision/dockingStationImages/station'+word+'.jpg')
    # Convert BGR to RGB (OpenCV uses BGR by default)
    images.append(img)
    RGB_images.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    # cv2.imshow(word,img)

# cv2.waitKey(0)

rows = 3
columns = 4

fig, axes = plt.subplots(nrows=rows, ncols=columns, figsize=(columns*3, rows*3))

for num in range(1, rows*columns+1):
    fig.add_subplot(rows, columns, num)
    
    idx = num - 1
    
    plt.imshow(images[idx], aspect='auto')
    
    if num % 3 == 1 : #if remainder is 1
        file_idx = num // 3 # get quotient
        plt.ylabel(f'{[file_idx]}', 
                  rotation=0,
                  fontsize=12,
                  labelpad=50) #add space between ylabel and yaxis
    
    
fig.tight_layout() # used to adjust padding between subplots 

plt.show()

# Make and create new array of images in grayscale
gray_images = []
for i in range(0, numImages-1):
    img = images[i]
    gray_images.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))



img1 = RGB_images[0]
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

