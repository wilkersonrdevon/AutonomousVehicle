#!/usr/bin/env python2.7

import math
from math import radians, sin, cos, pi

import rospy
from tf import TransformBroadcaster, transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped

rospy.init_node('odometry_publisher')

left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

def ticks_callback(msg):
    global left_ticks
    global right_ticks
    left_ticks = msg.vector.x
    right_ticks = msg.vector.y
    trash = msg.vector.z

ticks_sub = rospy.Subscriber("encoder_ticks", Vector3Stamped, callback=ticks_callback)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = TransformBroadcaster()

wheel_radius = 0.02
wheelbase = 0.225
ppr = 408
meter_per_tick = 1/10000
distance_per_count = (2 * pi * wheel_radius) / ppr

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    
    
    dlt = left_ticks - last_left_ticks
    drt = right_ticks - last_right_ticks

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()

    vl = (dlt * distance_per_count) / ((dt + 0.000001)*pi)
    vr = (drt * distance_per_count) / ((dt + 0.000001)*pi)

    vx = (vl+vr)/2
    xy = 0
    vth = (vl - vr)/ wheelbase

    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)
    last_left_ticks=left_ticks
    last_right_ticks=right_ticks
    last_time = current_time
    r.sleep()