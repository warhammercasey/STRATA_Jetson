import cv2
import os
import numpy as np
import itertools

MORPH_KERNEL_SIZE = 6 # Kernel size for morphological opening

CIRCLE_WEIGHT = 1.5
WHITE_WEIGHT = 1
DIST_WEIGHT = 2

# Camera information
FOV = 62.2
H_RESOLUTION = 640
V_RESOLUTION = 480
FRAMERATE = 30


# 3D points of LEDs
target_points = np.array([
    (0, 0, 0),
    (100, 0, 0),
    (50, 10, 0),
    (75, 25, 0),
    (90, 50, 0),
    (50, 90, 0),
    (0, 100, 0),
    (100, 100, 0)
], dtype='float32')

#target_points = np.array([
#    (0, 100, 0),
#    (100, 100, 0),
#    (50, 90, 0),
#    (75, 75, 0),
#    (90, 50, 0),
#    (50, 10, 0),
#    (0, 0, 0),
#    (100, 0, 0)
#], dtype='float64')


# Make camera calibration matrix
f = H_RESOLUTION/2/np.arctan(FOV*np.pi/180/2)

calibration_matrix = np.array([
    (f, 0, H_RESOLUTION/2),
    (0, f, V_RESOLUTION/2),
    (0, 0, 1)
])
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=H_RESOLUTION,
    capture_height=V_RESOLUTION,
    display_width=H_RESOLUTION,
    display_height=V_RESOLUTION,
    framerate=FRAMERATE,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


# Gets the camera coordinates of all detected LEDs
# img - bgr input image
# target_length - the amount of LEDs to try to detect
# kernel_size - the size of the kernel used for morphological opening
# morph_iterations - the amount of times to do a morphological opening
# Gets the camera coordinates of all detected LEDs
# img - bgr input image
# target_length - the amount of LEDs to try to detect
# kernel_size - the size of the kernel used for morphological opening
# morph_iterations - the amount of times to do a morphological opening
def get_points(Ibgr, target_length, kernel_size = MORPH_KERNEL_SIZE, morph_iterations = 1):
    # Read image
    I = cv2.cvtColor(Ibgr, cv2.COLOR_BGR2GRAY)

    # Threshold image
    (thresh, im_bw) = cv2.threshold(I, np.max(I) - 30, 255, cv2.THRESH_BINARY)

    # Morphologically open image to get rid of any noise
    y, x = np.ogrid[-MORPH_KERNEL_SIZE/2:MORPH_KERNEL_SIZE/2+1, -MORPH_KERNEL_SIZE:MORPH_KERNEL_SIZE+1]
    kernel = (x**2+y**2 <= (MORPH_KERNEL_SIZE/2)**2).astype(np.uint8)
    #kernel = np.ones((kernel_size, kernel_size), np.uint8)
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
            cv2.imwrite("Base Image.png", Ibgr)
            cv2.imwrite("Binary Image.png", im_bw)
            print("Wrote image")
            print("Failed")
            return ([], True, [])
        return get_points(Ibgr, target_length, kernel_size = kernel_size - 1, morph_iterations = morph_iterations)
    

    # Calculate the circularity of each blob based on https://sciencing.com/calculate-circularity-5138742.html
    circular = np.zeros((num_labels))

    for i in range(num_labels):
        mask = np.array(labels == i+1, dtype='uint8')*255

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) <= 0:
            continue

        circular[i] = 4*np.pi*stats[i, cv2.CC_STAT_AREA]/(cv2.arcLength(contours[0], True)**2)

    circular[circular > 1] = 0



    # Calculate how white each blob is by converting to HSV color space
    whiteness = np.zeros((num_labels))

    hsv = cv2.cvtColor(Ibgr, cv2.COLOR_BGR2HSV)

    for i in range(num_labels):
        pos = np.round(centroids[i]).astype(np.int32)
        
        whiteness[i] = (255 - hsv[pos[1]][pos[0]][1])/255

    # Find outliers by calculating the median x/y coordinate of the blobs and getting the distance to that median
    distance = np.zeros((num_labels))
    median_count = np.min([num_labels, np.round(target_length*1.5)]).astype(np.int32)

    x_points = [centroids[x][0] for x in circular.argsort()[-median_count:]]
    y_points = [centroids[x][1] for x in circular.argsort()[-median_count:]]

    median_x = np.median(x_points)
    median_y = np.median(y_points)

    for i in range(num_labels):
        distance[i] = np.sqrt((centroids[i][0] - median_x)**2 + (centroids[i][0] - median_y)**2)

    distance = (np.max(distance) - distance)/np.max(distance)


    # Give each blob a score based on circularity, whiteness, and distance from median
    detected = circular*CIRCLE_WEIGHT + whiteness*WHITE_WEIGHT + distance*DIST_WEIGHT

    # Calculate a confidence score based on the mean score of detected dots
    confidence = np.mean(detected[detected.argsort()[-target_length:]])

    # Get the centroids of the detected blobs
    detected = centroids[detected.argsort()[-target_length:]]
    
    # Draw dots on each detected led
    for i in detected:
        Ibgr = cv2.circle(Ibgr, (int(np.round(i[0])), int(np.round(i[1]))), radius=5, color=(0,0,255), thickness=-1)

    ## Show images
    '''cv2.imshow("Base Image", Ibgr)
    cv2.imshow("Binary Image", im_bw)
    cv2.waitKey(0)'''
    
    cv2.imwrite("Base Image.png", Ibgr)
    cv2.imwrite("Binary Image.png", im_bw)
    print("Wrote image")
    
    return (detected, False, confidence)

if __name__ == "__main__":
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

    if video_capture.isOpened():
        try:
            ret_val, frame = video_capture.read()
            _, _, confidence = get_points(frame, len(target_points))
            print("Confidence: ", confidence)
        finally:
            video_capture.release()
    else:
        print("Unable to open camera")
  
