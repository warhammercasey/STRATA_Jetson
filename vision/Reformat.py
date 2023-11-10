import cv2
import os
import numpy as np
import itertools

CIRCLE_THRESHOLD = 0.01 # How tolerant of things being circles. Higher values will detect more objects as circular
MORPH_KERNEL_SIZE = 10 # Kernel size for morphological opening
MAX_MORPH_ITERATIONS = 10
WHITE_THRESHOLD = 5

# Camera information
FOV = 54
H_RESOLUTION = 640
V_RESOLUTION = 480

# Directory of images
WORKING_DIR = "C:\\Users\\mmurr\\Documents\\School\\Senior Design\\Computer Vision"

os.chdir(WORKING_DIR)


# 3D points of LEDs
target_points = np.array([
    (18.98, 3.36, -1),
    (48.76, 14.17, -1),
    (40.36, 28.76, -1),
    (33.61, 44.68, -1)
], dtype="float64")


# Make camera calibration matrix
f = H_RESOLUTION/2/np.arctan(FOV*np.pi/180/2)

calibration_matrix = np.array([
    (f, 0, H_RESOLUTION/2),
    (0, f, V_RESOLUTION/2),
    (0, 0, 1)
])


# Gets the camera coordinates of all detected LEDs
# img - filename of input image
# target_length - the amount of LEDs to try to detect
# kernel_size - the size of the kernel used for morphological opening
# morph_iterations - the amount of times to do a morphological opening
def get_points(img, target_length, kernel_size = MORPH_KERNEL_SIZE, morph_iterations = 1):
    # Read image
    Ibgr = cv2.imread(img)
    I = cv2.cvtColor(Ibgr, cv2.COLOR_BGR2GRAY)

    # Threshold image
    (thresh, im_bw) = cv2.threshold(I, np.max(I) - 50, 255, cv2.THRESH_BINARY)

    # Morphologically open image to get rid of any noise
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    im_bw = cv2.morphologyEx(im_bw, cv2.MORPH_OPEN, kernel, iterations=morph_iterations)

    # Calculated connected blobs
    connected = cv2.connectedComponentsWithStats(im_bw, 4, cv2.CV_32S)

    num_labels = connected[0] - 1
    labels = connected[1]
    stats = connected[2][1:]
    centroids = connected[3][1:]
    
    
    
    # Dynamically reduce kernel size if we dont detect enough dots
    if num_labels < target_length:
        if kernel_size == 0:
            return []
        return get_points(img, target_length, kernel_size = kernel_size - 1, morph_iterations = morph_iterations)
    '''elif num_labels > target_length + 1:
        if morph_iterations >= MAX_MORPH_ITERATIONS:
            return []
        return get_points(img, target_length, kernel_size = kernel_size, morph_iterations = morph_iterations + 1)'''

    # Figure out which blobs are circular
    circular = [abs(((stats[i, cv2.CC_STAT_HEIGHT] + stats[i, cv2.CC_STAT_WIDTH])/4)**2*np.pi/stats[i, cv2.CC_STAT_AREA] - 1) <= CIRCLE_THRESHOLD for i in range(num_labels)]

    # Increase thershold until enough circular blobs are detected
    thresh = CIRCLE_THRESHOLD
    while sum(circular) < target_length:
        thresh += 0.01
        if thresh > 1:
            return []
        circular = [abs(((stats[i, cv2.CC_STAT_HEIGHT] + stats[i, cv2.CC_STAT_WIDTH])/4)**2*np.pi/stats[i, cv2.CC_STAT_AREA] - 1) <= thresh for i in range(num_labels)]

    
    # Get rid of any non-white blobs
    white = circular[:]
    for i in range(len(circular)):
        if not circular[i]:
            continue
        
        pos = np.round(centroids[i]).astype(np.int32)
        avg = np.mean(Ibgr[pos[1]][pos[0]])
        
        white[i] = np.max(np.abs(np.array([avg, avg, avg]) - Ibgr[pos[1]][pos[0]])) <= WHITE_THRESHOLD
    
    # Increase white threshold until we have enough blobs detected
    thresh = WHITE_THRESHOLD
    while sum(white) < target_length:
        thresh += 1
        if thresh > 100:
            print("Threshold not found")
            return []
        white = circular[:]
        for i in range(len(circular)):
            if not circular[i]:
                continue
            
            pos = np.round(centroids[i]).astype(np.int32)
            avg = np.mean(Ibgr[pos[1]][pos[0]])
            
            white[i] = np.max(np.abs(np.array([avg, avg, avg]) - Ibgr[pos[1]][pos[0]])) <= thresh
        
    circular = white[:]
    
    # Draw dots on each detected led
    for i in range(len(circular)):
        if not circular[i]:
            continue
        
        Ibgr = cv2.circle(Ibgr, (int(round(centroids[i][0])), int(round(centroids[i][1]))), radius=5, color=(0,0,255), thickness=-1)

    # Show images
    cv2.imshow("Base Image", Ibgr)
    cv2.imshow("Binary Image", im_bw)
    cv2.waitKey(0)
    
    return centroids[circular]

IMG = 'img4.jpg'

#target_points = get_points('img0.jpg', len(target_points)) 
seen_points = get_points(IMG, len(target_points)).astype('float64')
#seen_points = np.array(seen_points, dtype='double')

success, rotation_vector, translation_vector = cv2.solvePnP(target_points, seen_points, calibration_matrix, np.zeros((4, 1)), flags=0)

print("Success: " + str(success))
print("rotation_vector: " + str(rotation_vector))
print("translation_vector: " + str(translation_vector))
