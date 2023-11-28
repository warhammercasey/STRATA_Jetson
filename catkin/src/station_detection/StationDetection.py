#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool, Float64MultiArray
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO
from scipy.spatial.transform import Rotation as R
import copy

MORPH_KERNEL_SIZE = 20 # Kernel size for morphological opening

CIRCLE_WEIGHT = 1.5
WHITE_WEIGHT = 1
DIST_WEIGHT = 0.5
MIN_CONFIDENCE = (CIRCLE_WEIGHT + WHITE_WEIGHT + DIST_WEIGHT)*2/3

DETECTION_COUNT = 10

# Camera information
FOV = 62.2
H_RESOLUTION = 1296
V_RESOLUTION = 972

FRAMERATE = 30

SHITTY_ROTATION = 5*np.pi/180 # degrees

DETECTION_TIMEOUT = 0.5 # seconds
AVG_FILTER_SIZE = 10

TURNING_GAIN = 1 # The rotation to the station is multiplied by this to get the turn angle

# PID gains
SPEED_KP = 0.05
SPEED_KI = 0.005
SPEED_KD = 0.005

# We will lose track of the station once we get too close
# Once we get to that point we start final guidance where we just go straight until we reach the station or we have gone too far
APPROACH_DISTANCE = 350 # If we lose track of the station below this distance, start final guidance
APPROACH_SPEED = 20 # Speed to approach the station at in final guidance

STATION_PIN = 21 # GPIO pin of jetson which the receiver of the station is connected to. This pin should go high when charging

WHEEL_DIAMETER = 90 # mm

position = [0, 0, 0, 0]

SAVE_VIDEO = False


# 3D points of LEDs
#target_points = np.array([
#    (18.98, 3.36, 0),
#    (48.76, 14.17, 0),
#    (40.36, 28.76, 0),
#    (33.61, 44.68, 0)
#], dtype="float64")

# 3D coordinates of LEDs
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

# So there was this big issue where I have no idea how to relate which detected LED coordinate in 2d pixel coordinates corresponds to which LED in 3d coordinates and thats needed for pose estimation.
# When the LEDs are detected, they are in a likely random order so we dont know which one is which.
# My (bad) solution to this is to create two sets of target points and rotate them around the z-axis just a little bit in opposite directions. They are then sorted by position
# When the LEDs are detected, those are also sorted by position in the same way and then pose estimation is run between the sorted points and both sets of rotated target points.
# The idea is that the detected points will likely be sorted such that they properly correspond with one of the rotated sets of points and then the pose estimation that had the least error would be the correct one
# The points are then reprojected into the image frame and the reprojection error is calculated to figure out which one is the most correct
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
            return ([], True)
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
    
    '''# Draw dots on each detected led
    for i in detected:
        Ibgr = cv2.circle(Ibgr, (int(np.round(i[0])), int(np.round(i[1]))), radius=5, color=(0,0,255), thickness=-1)

    ## Show images
    cv2.imshow("Base Image", Ibgr)
    cv2.imshow("Binary Image", im_bw)
    cv2.waitKey(0)'''
    
    return (detected, False, confidence)

def position_callback(data):
    global position
    position = data.data
    for i in position:
        i *= WHEEL_DIAMETER

if __name__ == "__main__":
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

    if SAVE_VIDEO:
        out = cv2.VideoWriter('/home/strata/output.avi', cv2.VideoWriter_fourcc(*"MJPG"), 30, (H_RESOLUTION, V_RESOLUTION))

    # Create node
    rospy.init_node('StationDetection')
    rospy.loginfo("Running")

    # Make sure the camera was successfully connected to
    if video_capture.isOpened():
        
        detection_count = 0

        # Setup the charge station input pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(STATION_PIN, GPIO.IN)

        # Setup ros
        rospy.Subscriber("wheelControl/position", Float64MultiArray, position_callback) # Gets the position of the motors in revolutions

        detected_pub = rospy.Publisher("StationDetection/station_detected", Bool, queue_size=100) # Publishes True when station is detected and False when not detected
        speed_pub = rospy.Publisher("wheelControl/target_speed", Float32, queue_size=100) # Publishes the target speed to the wheel controller
        angle_pub = rospy.Publisher("wheelControl/turn_angle", Float32, queue_size=100) # Publishes the target turn angle to the wheel controller


        # Main loop should be run at the framerate
        rate = rospy.Rate(FRAMERATE)


        last_detection = 0 # Timestamp of the last time the station was detected
        detected = False # Whether or not the station is currently detected

        # Array of previous distance and rotation values to be used in an averaging filter since the distances can sometimes be wonky
        previous_distance = [] 
        previous_rotation = []

        # Static variables for movement PID loop
        distance_integrator = 0
        last_distance_error = 0

        # Static variables for final guidance
        # The station wont be able to be detected once we are close enough to charge off of so we need to just blindly move forward either until we have gone too far or we are docked
        initial_position = position # Stores position of the motors at the moment the station went out of view
        final_guidance = False # True if we are in the final guidance state

        try:
            while not rospy.is_shutdown():
                # Wait until this is supposed to run again
                rate.sleep()

                # Read video frame
                ret_val, frame = video_capture.read()
                
                # Calculate the pixel coordinates of all LEDs in the image
                seen_points, failed, confidence = get_points(frame, len(target_points))
                
                if SAVE_VIDEO:

                    frame = cv2.putText(frame, "Confidence: " + str(confidence), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, 2)
                    for i in range(len(seen_points)):
                        frame = cv2.circle(frame, (int(np.round(seen_points[i][0])), int(np.round(seen_points[i][1]))), 5, (0, 0, 255), -1)
                    
                        frame = cv2.putText(frame, str(i), (int(seen_points[i][0]), int(seen_points[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, 2)
                    
                
                # Check if detection failed
                if (len(seen_points) != len(target_points)) or (failed) or (confidence < MIN_CONFIDENCE):
                    detection_count = 0
                    rospy.loginfo("Detection failed")

                    # If we havent seen the station in a while, assume we lost detection
                    if time.time() - last_detection > DETECTION_TIMEOUT:
                        # Reset PID loop
                        distance_integrator = 0
                        last_distance_error = 0

                        if detected: # If we just now lost detection
                            # Flag as lost detection
                            detected = False
                            rospy.loginfo("Lost detection")

                            # Save the position of the motors for final guidance
                            initial_position = position[:]

                            # Check if we are close enough to use final guidance
                            if previous_distance[-1] <= APPROACH_DISTANCE:
                                final_guidance = True
                            
                            # Tell ROS we no longer detect the station
                            data_to_send = Bool()
                            data_to_send.data = detected
                            detected_pub.publish(data_to_send)

                        if final_guidance: # If we dont see the station and we are in final guidance
                            rospy.loginfo(position)
                            rospy.loginfo("Final guidance: " + str(np.mean(position) - np.mean(initial_position)) + "\tPrevious distance: " + str(previous_distance[-1]))

                            # Check if we have docked or we moved too far
                            if GPIO.input(STATION_PIN) or np.mean(position) - np.mean(initial_position) > previous_distance[-1]*1.2:
                                # Disable final guidance
                                final_guidance = False

                                # Stop motors
                                data_to_send = Float32()
                                data_to_send.data = 0
                                speed_pub.publish(data_to_send)
                                angle_pub.publish(data_to_send)

                            else:
                                # Move straight
                                data_to_send = Float32()
                                data_to_send.data = 0
                                angle_pub.publish(data_to_send)

                                # Move at speed defined by APPROACH_SPEED
                                data_to_send = Float32()
                                data_to_send.data = APPROACH_SPEED
                                speed_pub.publish(data_to_send)

                        else: # We are not in final guidance, stop motors
                            data_to_send = Float32()
                            data_to_send.data = 0
                            speed_pub.publish(data_to_send)
                            angle_pub.publish(data_to_send)

                    # We didnt detect anything so skip handling data
                    if SAVE_VIDEO:
                        out.write(frame)
                    continue


                if not detected: # If this is the first time detecting the station
                    detection_count += 1
                    rospy.loginfo(detection_count)# Publish speed and turn data to wheel controller
                    
                    data_to_send = Float32()
                    data_to_send.data = 0
                    speed_pub.publish(data_to_send)

                    
                    data_to_send = Float32()
                    data_to_send.data = 0
                    angle_pub.publish(data_to_send)
                    
                    if detection_count > DETECTION_COUNT:
                        # Flag the station as detected
                        detected = True
                        rospy.loginfo("Station detected")
                        detection_count = 0

                        # Tell ros we found the station
                        data_to_send = Bool()
                        data_to_send.data = detected
                        detected_pub.publish(data_to_send)
                    else:
                        if SAVE_VIDEO:
                            out.write(frame)
                        continue

                # Update the last detection time and make sure final guidance is disabled
                last_detection = time.time()
                final_guidance = False
                
                # Sort the detected points to match up with the expected points
                seen_points = np.array(seen_points, dtype='float32')
                seen_points = seen_points[np.lexsort((seen_points[:,0], seen_points[:,1]))]
                
                # Solve pose estimation on both potential orders of points
                success = [0, 0]
                rotation_vector = [0, 0]
                translation_vector = [0, 0]
                success[0], rotation_vector[0], translation_vector[0] = cv2.solvePnP(target_rotated_1, seen_points, calibration_matrix, np.zeros((4, 1)), flags=cv2.SOLVEPNP_IPPE)
                success[1], rotation_vector[1], translation_vector[1] = cv2.solvePnP(target_rotated_2, seen_points, calibration_matrix, np.zeros((4, 1)), flags=cv2.SOLVEPNP_IPPE)

                if not any(success):
                    if SAVE_VIDEO:
                        out.write(frame)
                    continue

                # Figure out which pose estimation has the least reprojection error
                mean_error = [(not success[0])*100000, (not success[1])*100000]
                for i in range(len(seen_points)):
                    for j in range(2):
                        outputPoints, _ = cv2.projectPoints(target_rotated_1[i], rotation_vector[j], translation_vector[j], calibration_matrix, np.zeros((4, 1)))

                        error = cv2.norm(np.array(seen_points[i], dtype='float32'), np.array(outputPoints[0][0], dtype='float32'), cv2.NORM_L2)/len(outputPoints)

                        mean_error[j] += error/len(seen_points)

                min_idx = np.argmin(mean_error)
                distance = np.linalg.norm(translation_vector[min_idx])
                
                # THIS SHOULD BE IMPROVED
                # This just gets the rotation to the station by averaging the x coordinate of the LEDs and converting it to a rotation using the known field of view of the camera
                # It has a bunch of issues and the pose estimation rotation vector should be used but I'm not smart enough to figure that out
                rotation = (np.mean([i[0] for i in seen_points]) - H_RESOLUTION/2)/H_RESOLUTION*FOV
                

                # Convert the rotation vector from solvePnP to euler rotation
                # This isnt actually used in the current code but I want to keep it because this is what it should be doing
                '''rotation_matrix1 = cv2.Rodrigues(rotation_vector1)[0]
                rotation_matrix2 = cv2.Rodrigues(rotation_vector2)[0]
                rotation_vector1 = R.from_matrix(rotation_matrix1).as_rotvec()
                rotation_vector2 = R.from_matrix(rotation_matrix2).as_rotvec()
                
                # Convert to degrees
                for i in rotation_vector1:
                    i *= 180/np.pi
                    
                for i in rotation_vector2:
                    i *= 180/np.pi'''
                
                # Add distance and rotation to the averaging buffer
                previous_distance.append(distance)
                previous_rotation.append(rotation)

                # Make sure the averaging buffer doesnt get too long
                if len(previous_distance) > AVG_FILTER_SIZE:
                    previous_distance = previous_distance[-AVG_FILTER_SIZE:]

                if len(previous_rotation) > AVG_FILTER_SIZE:
                    previous_rotation = previous_rotation[-AVG_FILTER_SIZE:]

                # Calculate the average distance and rotation
                distance = np.mean(previous_distance)
                rotation = np.mean(previous_rotation)

                # PID
                distance_error = distance
                distance_integrator += distance

                speed = SPEED_KP*distance_error + SPEED_KI*distance_integrator/FRAMERATE + SPEED_KD*(distance_error - last_distance_error)*FRAMERATE
                last_distance_error = copy.copy(distance_error)
                
                rospy.loginfo("Distance: " + str(distance/25.4) + "\tRotation" + str(rotation))


                # Publish speed and turn data to wheel controller
                data_to_send = Float32()
                data_to_send.data = speed
                speed_pub.publish(data_to_send)

                
                data_to_send = Float32()
                data_to_send.data = rotation*TURNING_GAIN
                angle_pub.publish(data_to_send)


                

                #print((np.mean([i[0] for i in seen_points]) - H_RESOLUTION/2)/H_RESOLUTION*FOV)
                    
                if SAVE_VIDEO:
                    frame = cv2.putText(frame, "Distance 1: " + str(distance), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, 2)
                    
                    frame = cv2.putText(frame, "Rotation: " + str(rotation), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, 2)

                    frame = cv2.putText(frame, "Reprojection Error: " + str(mean_error[min_idx]), (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, 2)
                    
                    if (not success[0]) and (not success[1]):
                        frame = cv2.putText(frame, "FAILED", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, 2)
                
                    out.write(frame)
        finally:
            video_capture.release()
            GPIO.cleanup()
            if SAVE_VIDEO:
                out.release()
    else:
        rospy.logfatal("Unable to open camera")