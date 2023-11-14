import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

CIRCLE_THRESHOLD = 0.01 # How tolerant of things being circles. Higher values will detect more objects as circular
MORPH_KERNEL_SIZE = 10 # Kernel size for morphological opening
MAX_MORPH_ITERATIONS = 10
WHITE_THRESHOLD = 5

# Camera information
FOV = 62.2
H_RESOLUTION = 1296
V_RESOLUTION = 972

SHITTY_ROTATION = 5*np.pi/180 # degrees

# 3D points of LEDs
#target_points = np.array([
#    (18.98, 3.36, 0),
#    (48.76, 14.17, 0),
#    (40.36, 28.76, 0),
#    (33.61, 44.68, 0)
#], dtype="float64")

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

rotation_matrix = np.array([
    [np.cos(SHITTY_ROTATION), -np.sin(SHITTY_ROTATION), 0],
    [np.sin(SHITTY_ROTATION),  np.cos(SHITTY_ROTATION), 0],
    [0, 0, 1]
])

target_rotated_1 = np.transpose(np.matmul(rotation_matrix, np.transpose(target_points)))
target_rotated_1 = target_rotated_1[np.lexsort((target_rotated_1[:, 0], target_rotated_1[:, 1]))]

rotation_matrix = np.array([
    [np.cos(-SHITTY_ROTATION), -np.sin(-SHITTY_ROTATION), 0],
    [np.sin(-SHITTY_ROTATION),  np.cos(-SHITTY_ROTATION), 0],
    [0, 0, 1]
])


target_rotated_2 = np.transpose(np.matmul(rotation_matrix, np.transpose(target_points)))
target_rotated_2 = target_rotated_2[np.lexsort((target_rotated_2[:, 0], target_rotated_2[:, 1]))]

#print(target_rotated_1)


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
    framerate=30,
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
def get_points(Ibgr, target_length, kernel_size = MORPH_KERNEL_SIZE, morph_iterations = 1):
    # Read image
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
        return get_points(Ibgr, target_length, kernel_size = kernel_size - 1, morph_iterations = morph_iterations)
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
    #for i in range(len(circular)):
    #    if not circular[i]:
    #        continue
    #    
    #    Ibgr = cv2.circle(Ibgr, (int(round(centroids[i][0])), int(round(centroids[i][1]))), radius=5, color=(0,0,255), thickness=-1)
#
    ## Show images
    #cv2.imshow("Base Image", Ibgr)
    #cv2.imshow("Binary Image", im_bw)
    #cv2.waitKey(0)
    
    return centroids[circular]


video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*"MJPG"), 30, (H_RESOLUTION, V_RESOLUTION))

if video_capture.isOpened():
    try:
        while True:
            ret_val, frame = video_capture.read()
            
            seen_points = get_points(frame, len(target_points))
            
            for i in range(len(seen_points)):
                frame = cv2.circle(frame, (int(np.round(seen_points[i][0])), int(np.round(seen_points[i][1]))), 5, (0, 0, 255), -1)
               
                frame = cv2.putText(frame, str(i), seen_points[i], cv2.FONT_HERSHY_SIMPLEX, 1, (0, 0, 0), 1, 2)
                
            
            if len(seen_points) != len(target_points):
                print("Failed")
                continue
            
            seen_points = np.array(seen_points, dtype='float32')
            seen_points = seen_points[np.lexsort((seen_points[:,0], seen_points[:,1]))]
            
            
            success1, rotation_vector1, translation_vector1 = cv2.solvePnP(target_rotated_1, seen_points, calibration_matrix, np.zeros((4, 1)), flags=cv2.SOLVEPNP_IPPE)
            success2, rotation_vector2, translation_vector2 = cv2.solvePnP(target_rotated_2, seen_points, calibration_matrix, np.zeros((4, 1)), flags=cv2.SOLVEPNP_IPPE)
            
            rotation_matrix1 = cv2.Rodrigues(rotation_vector1)
            rotation_matrix2 = cv2.Rodrigues(rotation_vector2)
            
            rotation_vector1 = R.from_matrix(rotation_matrix1).as_rotvec()
            rotation_vector2 = R.from_matrix(rotation_matrix2).as_rotvec()
            
            for i in rotation_vector1:
                i *= 180/np.pi
                
            for i in rotation_vector2:
                i *= 180/np.pi
            
            if success1:
                print("Rotation_vector1: " + str(rotation_vector1) + "\ttranslation_vector1: " + str(translation_vector1))
            else:
                print("Failed1")
                
            
            if success2:
                print("Rotation_vector2: " + str(rotation_vector2) + "\ttranslation_vector2: " + str(translation_vector2))
            else:
                print("Failed2")
                
                
            out.write(frame)
    finally:
        video_capture.release()
        out.release()
else:
    print("Unable to open camera")