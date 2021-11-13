#!/usr/bin/env python3
import rospy
import time

from adafruit_motorkit import MotorKit
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# sets motor speed between [-1.0, 1.0]


def set_speed(value):
    max_pwm = 115.0
    speed = value
    motor.throttle = speed


# stops all motors
def all_stop():
    motor.throttle = 0

# simple string commands (left/right/forward/backward/stop)
def callback(msg):
    if msg.linear.x != 0:
        set_speed(msg.linear.x)
    elif msg.angular.z != 0 :
        set_speed(msg.angular.z)
    elif msg.linear.x == 0 and msg.angular.z == 0:
        all_stop()

# initialization
if __name__ == '__main__':
    motor = MotorKit()
    motor = motor.motor2
    all_stop()
    # setup ros node
    rospy.init_node('motor_right_front')
    sub = rospy.Subscriber(name='/cmd_vel', data_class=Twist, queue_size=1, callback=callback)
    rospy.spin()
    all_stop()

