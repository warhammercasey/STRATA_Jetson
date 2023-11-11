# EENG 437/507
# Nathan Wasniak & Matthew Murray 
# 11/10/2023

import cv2
import matplotlib.pyplot as plt

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

# fig, axes = plt.subplots(4, 5, figsize=(12, 4))

# for i in range(0,12):
#     axes[i].(RGB_images[i])
#     axes[i].set_title('Image '+str(i))
#     axes[i].axis('off')

# plt.show()

rows = 4
columns = 3

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

# for ax, col in zip(axes[0], cols):
#     ax.set_title(col)

# for idx, ax in enumerate(axes.flat):
#     ax.set_xticks([])
#     ax.set_yticks([])

plt.show()