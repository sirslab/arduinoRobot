#!/usr/bin/env python
import rospy
from arduino_robot.msg import VectorStamped
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, PoseStamped
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from math import pi, atan
import tf
import tf2_ros
rospy.init_node('robot_positioning')

mode = 'aruco' # or vicon
def rad_to_deg(ang):
    return ang * 180.0 / pi

target = None
def callback(data):
	if mode=='vicon':
		quaternion = (
		data.transform.rotation.x,
		data.transform.rotation.y,
		data.transform.rotation.z,
		data.transform.rotation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]
		yaw_deg = rad_to_deg(yaw)
		phi = yaw_deg
	elif mode=='aruco':
		quaternion = (
		data.transform.rotation[0],
		data.transform.rotation[1],
		data.transform.rotation[2],
		data.transform.rotation[3])
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]
		yaw_deg = rad_to_deg(yaw)
		phi = yaw_deg


	if target is None: return (None, None)
	theta = atan( (data.transform.translation[1] - target.y) / (data.transform.translation[0] - target.x) )
	if target.y < 0: theta += pi 
	
	br = tf2_ros.TransformBroadcaster()
	t2 = geometry_msgs.msg.TransformStamped()
	t2.transform.translation.x = data.transform.translation[0]
	t2.transform.translation.y = data.transform.translation[1]
	t2.transform.translation.z = data.transform.translation[2]
	q = quaternion_from_euler(0, 0, theta)
	t2.transform.rotation.x = q[0]
	t2.transform.rotation.y = q[1]
	t2.transform.rotation.z = q[2]
	t2.transform.rotation.w = q[3]
	t2.child_frame_id = "robot_steering"
	t2.header.frame_id = "rf0"
	t2.header.stamp = data.header.stamp
	br.sendTransform(t2)
	return phi, theta

def target_callback(data):
	global target
	target = Point(data.pose.position.x,data.pose.position.y,data.pose.position.z)

target_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, target_callback, queue_size=1)
if mode == 'vicon':
	sub = rospy.Subscriber("vicon/robot/robot", TransformStamped, callback, queue_size=1)
elif mode == 'aruco':
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
    	try:
    		(trans,rot) = listener.lookupTransform('/rf0', '/robot', rospy.Time(0))
    		ts = geometry_msgs.msg.TransformStamped()
    		ts.header.stamp = rospy.Time.now()
    		ts.header.frame_id = ""
    		ts.child_frame_id = ""
    		ts.transform.translation = trans
    		ts.transform.rotation = rot
    		callback(ts)
    	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    		pass


rospy.spin()
