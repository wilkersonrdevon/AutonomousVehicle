#!/usr/bin/env python3
import rospy
import time
import RPi.GPIO as gpio
from std_msgs.msg import String
from geometry_msgs.msg import Twist

gpio.setmode(gpio.BOARD)
left_power_forward = 21
left_power_reverse = 22
gpio.setup(left_power_forward, gpio.OUT, initial=gpio.LOW)
gpio.setup(left_power_reverse, gpio.OUT)
# stops all motors
def all_stop():
    gpio.output(left_power_forward, False)
    gpio.output(left_power_reverse, False)

# simple string commands (left/right/forward/backward/stop)
def callback(msg):
    if msg.linear.x != 0:
        gpio.output(left_power_forward, True)
    elif msg.angular.z != 0 :
        gpio.output(left_power_forward, True)
    elif msg.linear.x == 0 and msg.angular.z == 0:
        all_stop()
#motor 1 - -
#motor 2 + -
#motor 3 - +
#motor 4 + +
# initialization
if __name__ == '__main__':
    time.sleep(0.5)
    
    
    all_stop()
    # setup ros node
    rospy.init_node('motor_left')
    sub = rospy.Subscriber(name='/cmd_vel', data_class=Twist, queue_size=1, callback=callback)
    rospy.spin()
    all_stop()

