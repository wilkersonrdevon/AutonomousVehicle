#!/usr/bin/env python3
import rospy
import time

from adafruit_motorkit import MotorKit
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = value
	print(motor_ID)
	if motor_ID == motor_left:
		motor = motor_left
	elif motor_ID == motor_right:
		motor = motor_right
	motor.throttle = speed

# stops all motors
def all_stop():
	motor_left.throttle = 0
	motor_right.throttle = 0

# simple string commands (left/right/forward/backward/stop)
def callback(msg):
    if msg.linear.x != 0:
        set_speed(motor_left, msg.linear.x)
        set_speed(motor_right, msg.linear.x)
    elif msg.angular.z != 0 :
        set_speed(motor_left, -msg.angular.z)
        set_speed(motor_right, msg.angular.z)
    elif msg.linear.x == 0 and msg.angular.z == 0:
        all_stop()

# initialization
if __name__ == '__main__':
    motor = MotorKit()
    motor_left = motor.motor1
    motor_right = motor.motor2
    all_stop()
    # setup ros node
    rospy.init_node('twist_sub')
    sub = rospy.Subscriber('/cmd_vel', Twist, queue_size=1, callback=callback)
    rospy.spin()
    all_stop()

