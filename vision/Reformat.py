import cv2
import os
import numpy as np
import itertools

CIRCLE_THRESHOLD = 0.01 # How tolerant of things being circles. Higher values will detect more objects as circular
MORPH_KERNEL_SIZE = 10 # Kernel size for morphological opening
MAX_MORPH_ITERATIONS = 10
WHITE_THRESHOLD = 5

WORKING_DIR = "C:\\Users\\mmurr\\Documents\\School\\Senior Design\\Computer Vision"

os.chdir(WORKING_DIR)


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

    thresh = CIRCLE_THRESHOLD
    while sum(circular) < target_length:
        thresh += 0.01
        if thresh > 1:
            return []
        circular = [abs(((stats[i, cv2.CC_STAT_HEIGHT] + stats[i, cv2.CC_STAT_WIDTH])/4)**2*np.pi/stats[i, cv2.CC_STAT_AREA] - 1) <= thresh for i in range(num_labels)]

    
    white = circular[:]
    for i in range(len(circular)):
        if not circular[i]:
            continue
        
        pos = np.round(centroids[i]).astype(np.int32)
        avg = np.mean(Ibgr[pos[1]][pos[0]])
        
        white[i] = np.max(np.abs(np.array([avg, avg, avg]) - Ibgr[pos[1]][pos[0]])) <= WHITE_THRESHOLD
    
    
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

    cv2.imshow("Base Image", Ibgr)
    cv2.imshow("Binary Image", im_bw)
    cv2.waitKey(0)
    
    return centroids[circular]

IMG = 'img6.jpg'

target_points = get_points('img0.jpg', 4)  
seen_points = get_points(IMG, 4)

N = len(target_points)

A = np.zeros((2*N, 4))

for i in range(N):
    A[2*i]     = [target_points[i][0], -target_points[i][1], 1, 0]
    A[2*i + 1] = [target_points[i][1],  target_points[i][0], 0, 1]
    
permutations = list(itertools.permutations(seen_points, len(target_points)))
best_mag = 0
best_M = 0

for i in range(len(permutations)):
    M = cv2.getPerspectiveTransform(np.array([target_points], np.float32), np.array([permutations[i]], np.float32))
    #print(M)
    
    #mag = abs(np.arccos(M[0][0]) + np.arcsin(M[1][0]))/2
    mag = np.sqrt(M[0][2]**2 + M[1][2]**2)
    
    if best_mag == 0:
        best_M = M
    
    if (best_mag == 0 or mag < best_mag) and M[0][0] >= 0 and M[1][1] >= 0:
        best_M = M
        best_mag = mag

I = cv2.imread(IMG)

warped = cv2.warpPerspective(I, best_M, (len(I), len(I[0])))

cv2.imshow("Original", I)
cv2.imshow("warped", warped)
cv2.waitKey(0)