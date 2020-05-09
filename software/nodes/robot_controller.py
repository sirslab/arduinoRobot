#!/usr/bin/env python
# -*- coding: utf-8 -*- 

s = 0.4
p1_r = -6.449e-06
p2_r = 0.005872
p3_r = 0.08566

p1_s = -0.4448
p2_s = 4.029
p3_s = 0.1434

angle_low_threshold = 15 #degrees
angle_high_threshold = 20 #degrees
minimum_rotation_runtime = 0.15 #seconds

length_low_threshold = 0.1 #meters
minimum_translation_runtime = 0.15 #seconds
maximum_translation_runtime = 3.0 #seconds
length_error_threshold = 0.08 #meters

sleep_time = 3.0 #seconds

import rospy
from mob_robots.msg import VectorStamped

from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Pose, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import tf.transformations

from math import pi, atan, exp
import numpy as np
import tf
rospy.init_node('robot_controller')
speed_pub = rospy.Publisher("/commands", VectorStamped, queue_size=200)

target = None

marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)                                                   
mid = 0
trans = None
rot = None
moving = False

def show_text_in_rviz(marker_publisher, text, scale = 0.06):
    global mid
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(15),
                pose=Pose(Point(0.5, 0.5, 0.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(scale, scale, scale),
                header=Header(frame_id='robot'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)
    mid += 1
    
def print_(s):
	print("[controller] ", s)
	
def rad_to_deg(ang):
    return ang * 180.0 / pi
    
def deg_to_rad(ang):
    return ang *  pi / 180.0

def go_straight():
    global moving
    
    print_("going straight")
    try:
        (position,orientation) = listener.lookupTransform('/rf0', '/robot', rospy.Time(0))
        mat = tf.transformations.quaternion_matrix(orientation)
        robot_direction_xy = mat[:-2, 0]
        print("mat", mat)
        print("robot_direction", robot_direction_xy)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    delta_length = np.linalg.norm([position[0] - target.x, position[1] - target.y])

    
    target_xy = np.asarray([target.x, target.y])
    print("target_xy", target_xy)
    position_xy = np.asarray(position[:-1])
    print("position_xy", position_xy)
    delta_s_xy = target_xy - position_xy
    print("delta_s_xy", delta_s_xy)
    
    # Determining forward / backward motion
    cosine_ang = np.dot(delta_s_xy, robot_direction_xy) / (np.linalg.norm(delta_s_xy) * np.linalg.norm(robot_direction_xy))
    if cosine_ang < 0: s_ = -s
    else: s_ = s
    
    # Determining command duration
    if delta_length < length_low_threshold: runtime = minimum_translation_runtime
    else: runtime = p1_s* delta_length**2 + p2_s * delta_length + p3_s
    runtime = min(runtime, maximum_translation_runtime)
    
    moving = True
    v = VectorStamped()
    v.header.stamp = rospy.Time.now()
    v.header.frame_id = ""
    v.data = [s_, s_, runtime]
    print_(v)
    speed_pub.publish(v)
    print("Delta length: ", delta_length, "running for ", runtime, "with speed", s_)
    rospy.sleep(sleep_time)
    
    moving = False
    print "-----------"

def make_step(delta):
    global moving
    
	
    delta_deg = rad_to_deg(delta)
    delta_deg_abs = abs(delta_deg)
    if delta_deg > 90:
        delta_deg -= 180
    elif delta_deg < -90:
        delta_deg += 180

	delta_deg_abs = abs(delta_deg)
    
    
    try:
        (position,_) = listener.lookupTransform('/rf0', '/robot_steering', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    
    delta_length = np.linalg.norm([position[0] - target.x, position[1] - target.y])
    #print("Delta deg",delta_deg, "Delta length", delta_length)
    
    if delta_length < length_error_threshold:
		show_text_in_rviz(marker_publisher, "Target reached", scale = 0.1)
		return
		
    if delta_deg_abs < angle_low_threshold:
        return go_straight()
    else:
        show_text_in_rviz(marker_publisher, "Angle error: "+str(int(delta_deg))+"°")
        if delta_deg_abs > angle_high_threshold:
            runtime = p1_r* delta_deg_abs**2 + p2_r * delta_deg_abs + p3_r
        else: runtime = minimum_rotation_runtime
        
        moving = True
        v = VectorStamped()
        v.header.stamp = rospy.Time.now()
        v.header.frame_id = ""
        if delta < 0:
            v.data = [s, -s, runtime]
            print("Going clockwise!")
        else:
            v.data = [-s, s, runtime]
            print("Going counterclockwise!")
        speed_pub.publish(v)
		
        print("Delta deg",delta_deg, " -> running for ", runtime)
        rospy.sleep(sleep_time)
        moving = False
        
def target_callback(data):
	global target
	target = Point(data.pose.position.x,data.pose.position.y,data.pose.position.z)

target_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, target_callback, queue_size=1)


listener = tf.TransformListener()
while not rospy.is_shutdown():
	if target is None or moving: continue
	try:
		(trans,rot) = listener.lookupTransform('/robot', '/robot_steering', rospy.Time(0))
		delta_angle = tf.transformations.euler_from_quaternion(rot)[-1]
		make_step(delta_angle)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass
rospy.spin()
