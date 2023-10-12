import serial
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, UInt8, Bool, Float64MultiArray
from geometry_msgs.msg import Vector3
import struct

SERIAL_PORT = '/dev/ttyTHS1'

# COMMANDS
SET_WHEEL_SPEED = bytes(b'\x01') # 4 double array of speeds (rpm)
STOP_WHEELS = bytes(b'\x02') # No args
GET_WHEEL_SPEED = bytes(b'\x03') # Returns 4 double array of speeds (rpm)
GET_WHEEL_POSITION = bytes(b'\x04') # Returns 4 double array of position (revol

RETURN_WHEEL_SPEED = bytes(b'\x05')
RETURN_WHEEL_POSITION = bytes(b'\x06')

target_speed = 0
target_turn = 0

def speedCallback(data):
    global target_speed
    target_speed = data.data
    
def turnCallback(data):
    global target_turn
    target_turn = data.data

if __name__ == "__main__":
    # Initialize node
    rospy.init_node('wheelControl')
    rospy.loginfo("Running")
    
    rospy.Subscriber("wheelControl/target_speed", Float32, speedCallback)
    rospy.Subscriber("wheelControl/turn_angle", Float32, turnCallback)
    
    positionPub = rospy.Publisher('wheelControl/position', Float64MultiArray)
    speedPub = rospy.Publisher('wheelControl/speed', Float64MultiArray)
    
    ser = serial.Serial(SERIAL_PORT, 115200)
    
    rate = rospy.rate(10)
    
    last_wheel_speed = [0]*4
    last_turn = 0
    while not rospy.is_shutdown():
        ser.write(GET_WHEEL_SPEED)
        ser.write(GET_WHEEL_POSITION)
        
        while ser.in_waiting:
            cmd = ser.read(1)
            
            wheel_speeds = [0]*4
            wheel_position = [0]*4
            
            if(cmd == RETURN_WHEEL_SPEED):
                data = ser.read(8*4)
                
                for i in range(4):
                    wheel_speeds[i] = struct.unpack("d", data[i*8:i*8+7])
                    
                    data_to_send = Float64MultiArray()
                    data_to_send.data = wheel_speeds
                    speedPub.publish(data_to_send)
                    
            if(cmd == RETURN_WHEEL_POSITION):
                data = ser.read(8*4)
                
                for i in range(4):
                    wheel_position[i] = struct.unpack("d", data[i*8:i*8+7])
                    
                    data_to_send = Float64MultiArray()
                    data_to_send.data = wheel_position
                    positionPub.publish(data_to_send)

                    
        
        if (wheel_speeds != last_wheel_speed) and (target_turn != last_turn):
            continue
        
        wheel_speeds = [target_speed]*4
        wheel_speeds[0:2:2] += target_turn
        wheel_speeds[1:2:3] -= target_turn
        
        # Set wheel speed
        if target_speed == 0:
            ser.write(STOP_WHEELS)
        else:
            data = bytearray(SET_WHEEL_SPEED)
            for i in wheel_speeds:
                data.extend(struct.unpack("d", i))
                
            ser.write(data)
            
        rate.sleep()
        
    ser.close()