#!/usr/bin/env python2
import sys
import rospy
import time
from radariq import RadarIQ, MODE_POINT_CLOUD, RESET_REBOOT, DENSITY_VERY_DENSE, DENSITY_NORMAL 
 

## General Info
# Data output as a list [[x, y, z, intensity, velocity]..]
# 

def point_cloud():
    rospy.init_node('Radar')
    print("Point Cloud")
    rate = rospy.Rate(10)
    try:
        riq = RadarIQ()
        riq.set_mode(MODE_POINT_CLOUD)
        riq.set_units('m', 'm/s') 
        riq.set_frame_rate(5)
        riq.set_distance_filter(0, 10)
        riq.set_angle_filter(-45, 45)
       # riq.set_point_density(DENSITY_VERY_DENSE)
       # riq.set_point_density(DENSITY_NORMAL)
        riq.start()

#time.sleep(10)

        print("Getting frame")
        frame = []
        # Continuously reads and outputs point cloud data
        for row in riq.get_data():
            if rospy.is_shutdown():
                break
            if frame is not None:
                print(row)
            else:
                print("None\n")
            
        riq.stop()
    except Exception as error:
        print(error)
#print(frame)
#print(len(frame))
#print(len(frame[0]))
        
    exit()
    print("Stopped")
if __name__ == '__main__':
    point_cloud()
