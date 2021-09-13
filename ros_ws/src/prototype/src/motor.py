#!/usr/bin/env python3
import rospy
import time

from adafruit_motorkit import MotorKit
from std_msgs.msg import String

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = value
	print(motor_ID)
	if motor_ID == motor_left:
		motor = motor_left
	elif motor_ID == motor_right:
		motor = motor_right
	# else:
	# 	rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
	# 	return
	
	motor.throttle = speed

	# if value > 0:
	# 	motor.run(MotorKit.FORWARD)
	# else:
	# 	motor.run(MotorKit.BACKWARD)


# stops all motors
def all_stop():
	motor_left.throttle = 0
	motor_right.throttle = 0


# directional commands (degree, speed)
def on_cmd_dir(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		set_speed(motor_left,  -1.0)
		set_speed(motor_right,  1.0) 
	elif msg.data.lower() == "right":
		set_speed(motor_left,   1.0)
		set_speed(motor_right, -1.0) 
	elif msg.data.lower() == "forward":
		set_speed(motor_left,   1.0)
		set_speed(motor_right,  1.0)
	elif msg.data.lower() == "backward":
		set_speed(motor_left,  -1.0)
		set_speed(motor_right, -1.0)  
	elif msg.data.lower() == "stop":
		all_stop()
	# else:
	# 	rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# initialization
if __name__ == '__main__':

	# setup motor controller
	motor = MotorKit()

	motor_left = motor.motor1
	motor_right = motor.motor2

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
	rospy.Subscriber('~cmd_raw', String, on_cmd_raw)
	rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()

