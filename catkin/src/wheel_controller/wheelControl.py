#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, UInt8, Bool
from scipy import signal

PPR = 6*30.0 # Encoder pulses per revolution
MAX_RPM = 11000/30.0 # Motor max rpm (after gearbox)

filterFs = 210.0 # Sample rate at which to run the motor speed filter (Hz)

FREQUENCY = 100.0 # Frequency to run the main PWM loop (Hz)
period = 1/FREQUENCY # PWM period

speed = 0 # Current PWM duty cycle
targetSpeed = 0 # Target RPM of the motor

# Pin definitions
pwmPin = 12
phasePin = 13
encA = 22
encB = 21

motorPosition = 0 # Current motor position (revolutions)
lastTime = 0.0 # Timestamp of the last encoder edge
motorSpeed = 0 # Raw motor speed from encoder
filteredSpeed = 0 # Motor speed after filtering
lastPosition = 0.0

# Filter constants
filterB = []
filterZ = 0

# PID loop gains
kp = 0.8
ki = 0.4
kd = 0.0

# PID loop static variables since python doesnt allow static variables
integrator = 0
lastError = 0

# onShutdown is called when the ros node receives the shutdown signal. All it really does is clean up GPIO settings
def onShutdown():
    GPIO.output(pwmPin, GPIO.LOW)
    GPIO.output(phasePin, GPIO.LOW)
    GPIO.cleanup()

# this is called at a rate defined by filterFs
# this runs the raw speed from the encoder through a digital lowpass filter with a cutoff frequency defined by FREQUENCY
# this smooths out any glitches with the encoder
def filterCallback(event):
    global filterB, filterZ, filteredSpeed, motorSpeed, lastPosition
    motorSpeed = (motorPosition - lastPosition)*filterFs*60
    lastPosition = motorPosition
    #filteredSpeed, filterZ = signal.lfilter(filterB, 1, [motorSpeed], zi=filterZ)
    filteredSpeed = (10*filteredSpeed + motorSpeed)/11.0

# PIDCallback is run at a rate defined by FREQUENCY
# This uses a PID loop to control the input PWM of the motor in order to accurately control its speed
# It still needs to be tuned
def PIDCallback(event):
    global integrator, lastError, filteredSpeed, speed, targetSpeed
    error = targetSpeed - filteredSpeed
    integrator += error*period
    derror = (error - lastError)/period

    lastError = error
    speed = kp*error + ki*integrator + kd*derror
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    

# encACallback is run whenver the value of the encA pin changes level
# It increments the motorPosition variable and sets the motorSpeed variable based on the time since the last encoder pulse
def encACallback(channel):
    global motorPosition , lastTime, motorSpeed
    if GPIO.input(encA) == GPIO.input(encB):
        motorPosition += 1/PPR

    else:
        motorPosition -= 1/PPR


# encBCallback is run whenver the value of the encB pin changes level
# It does the same exact thing as encA but reversed
def encBCallback(channel):
    global motorPosition , lastTime, motorSpeed
    if GPIO.input(encA) == GPIO.input(encB):
        motorPosition -= 1/PPR

    else:
        motorPosition += 1/PPR


# speedCallback is called whenever a message is published to the wheelControl/target_speed topic
# This will set the target speed of the motor
def speedCallback(data):
    global targetSpeed
    newSpeed = data.data

    if newSpeed > MAX_RPM:
        newSpeed = MAX_RPM
    elif newSpeed < -MAX_RPM:
        newSpeed = -MAX_RPM

    targetSpeed = newSpeed

    rospy.loginfo("Setting new speed: " + str(newSpeed))

# main function loop
def main():
    global speed, motorPosition, motorSpeed, filterB, filterZ, filteredSpeed

    # Setup GPIO pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pwmPin, GPIO.OUT)
    GPIO.setup(phasePin, GPIO.OUT)
    GPIO.setup(encA, GPIO.IN)
    GPIO.setup(encB, GPIO.IN)

    # Initialize node
    rospy.init_node('wheelControl')
    rospy.loginfo("Running")

    # Create ros publishers and subscribers
    rospy.Subscriber("wheelControl/target_speed", Float32, speedCallback)
    speedPub = rospy.Publisher('wheelControl/motor_speed', Float32)
    positionPub = rospy.Publisher('wheelControl/motor_position', Float32)

    # Let the user know the node is starting
    rospy.loginfo("Start while loop")
    rospy.loginfo("Period: " + str(period))

    # Create the lowpass filter to filter the raw speed from the encoders
    # 256 taps but that can be changed depending on needs
    # Cutoff frequency of FREQUENCY since any higher frequencies would be ignored/aliased by the PID loop
    filterB = signal.firwin(64, FREQUENCY/2.0, fs=filterFs)
    filterZ = signal.lfilter_zi(filterB, 1)
    rospy.Timer(rospy.Duration(1/filterFs), filterCallback)

    # Initialize the PID loop to run at FREQUENCY
    rospy.Timer(rospy.Duration(period), PIDCallback)

    # Create interrupt handlers for the encoder pins
    GPIO.add_event_detect(encA, GPIO.BOTH, callback=encACallback)
    GPIO.add_event_detect(encB, GPIO.BOTH, callback=encBCallback)


    # Main software PWM loop which runs at FREQUENCY
    while not rospy.is_shutdown():
        # Publish the current speed and position
        speedPub.publish(filteredSpeed)
        positionPub.publish(motorPosition)

        rospy.loginfo("Filtered speed: " + str(filteredSpeed) + "\tcurrent Position: " + str(motorPosition) + "\tPWM:" + str(speed))

        GPIO.output(phasePin, speed < 0)
        speed = abs(speed)
        # Handle the 0% and 100% duty edge cases
        if speed == 0:
            GPIO.output(pwmPin, GPIO.LOW)
            rospy.sleep(period)
            continue
        elif speed == 100:
            GPIO.output(pwmPin, GPIO.HIGH)
            rospy.sleep(period)
            continue


        # Software PWM
        GPIO.output(pwmPin, GPIO.HIGH)

        rospy.sleep(period*speed/100)
        GPIO.output(pwmPin, GPIO.LOW)

        rospy.sleep(period*(1 - speed/100))

    rospy.loginfo("Done") 

        
        


if __name__ == "__main__":
    main()
