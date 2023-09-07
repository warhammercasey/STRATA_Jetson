#!/usr/bin/env python2

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, UInt8, Bool

FREQUENCY = 10.0
period = 1/FREQUENCY
speed = 0
stop = False

pin_num = 12
encA = 22
encB = 21

motorPosition = 0
lastTime = 0.0
motorSpeed = 0

def onShutdown():
    GPIO.cleanup()

def encACallback(channel):
    global motorPosition
    global lastTime
    global motorSpeed
    now = rospy.get_time()
    if GPIO.input(encA) == GPIO.input(encB):
        motorPosition += 1
        motorSpeed = 1/(now - lastTime)
    else:
        motorPosition -= 1
        motorSpeed = -1/(now - lastTime)
    lastTime = now


def encBCallback(channel):
    global motorPosition 
    global lastTime
    global motorSpeed
    now = rospy.get_time()
    if GPIO.input(encA) == GPIO.input(encB):
        motorPosition -= 1
        motorSpeed = -1/(now - lastTime)
    else:
        motorPosition += 1
        motorSpeed = 1/(now - lastTime)
    lastTime = now


def speedCallback(data):
    global speed
    newSpeed = data.data

    if newSpeed > 100:
        newSpeed = 100
    elif newSpeed < 0:
        newSpeed = 0
    speed = newSpeed
    rospy.loginfo("Setting new speed: " + str(newSpeed))

def main():
    global speed
    global motorPosition 
    global motorSpeed

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin_num, GPIO.OUT)
    GPIO.setup(encA, GPIO.IN)
    GPIO.setup(encB, GPIO.IN)

    rospy.init_node('wheelControl')
    rospy.loginfo("Running")

    rospy.Subscriber("motor_speed", Float32, speedCallback)

    rospy.loginfo("Start while loop")
    rospy.loginfo("Period: " + str(period))
    lastSpeed = speed



    GPIO.add_event_detect(encA, GPIO.BOTH, callback=encACallback)
    GPIO.add_event_detect(encB, GPIO.BOTH, callback=encBCallback)
    while not rospy.is_shutdown():
        if lastSpeed != speed:
            rospy.loginfo("Changing speed to: " + str(speed))
            lastSpeed = speed

        if speed == 0:
            GPIO.output(pin_num, GPIO.LOW)
            rospy.sleep(period)
            continue
        elif speed == 100:
            GPIO.output(pin_num, GPIO.HIGH)
            rospy.sleep(period)
            continue

        rospy.loginfo("Position: " + str(motorPosition) + " Speed: " + str(motorSpeed))
        
        GPIO.output(pin_num, GPIO.HIGH)

        rospy.sleep(period*speed/100)
        GPIO.output(pin_num, GPIO.LOW)

        rospy.sleep(period*(1 - speed/100))

    rospy.loginfo("Done") 

        
        


if __name__ == "__main__":
    main()
