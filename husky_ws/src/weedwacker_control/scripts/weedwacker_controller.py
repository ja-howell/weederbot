#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import Joy

weedwacker = serial.Serial("/dev/ttyACM0", 19200, timeout = 0)

firstSpeed = None
calibrated = False
unlatched = False
enabled = False

lastSpeed = 0
cruiseControl = False

def callback(data):
    global firstSpeed
    global lastSpeed
    global cruiseControl
    global enabled

    speed = (data.axes[5] * -1 + 1)/2.0

    if not enabled:
        if firstSpeed == None:
            firstSpeed = speed
        elif speed != firstSpeed:
            enabled = True

        speed = 0

    else:
        if data.buttons[2]:

            if cruiseControl is not False:
                if speed > lastSpeed:
                    cruiseControl = False
                else:
                    lastSpeed = speed
                    speed = cruiseControl

            elif data.buttons[3]:
                cruiseControl = speed
                lastSpeed = speed
        else:
            speed = 0
            lastSpeed = 0
            cruiseControl = False

    i = int(speed * 254)
    print(i)
    weedwacker.write('d '+ str(i) + '\r')

    
def listener():
    rospy.init_node('weedwacker_controller', anonymous=True)

    rospy.Subscriber("/joy_teleop/joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
