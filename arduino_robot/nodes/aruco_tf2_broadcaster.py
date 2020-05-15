#!/usr/bin/env python 

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import pi
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Polygon, PolygonStamped

exploration_area_vertices = [Point(0.35,0.75,0.0), Point(0.8,0.4,0.0), Point(0.8,-0.35, 0.0), Point(0.37,-0.67,0.0)]


target = None
marker_id = 0
pubmarker = rospy.Publisher('target', Marker, queue_size=100)
pubarea = rospy.Publisher('exploration_area', PolygonStamped, queue_size=100)

def createArea(frame):
	poly = PolygonStamped()
	poly.header.frame_id = frame
	poly.header.stamp = rospy.Time.now()
	poly.polygon = Polygon()
	poly.polygon.points = exploration_area_vertices
	return poly
	
def target_callback(data):
	print("Got target!")
	global target
	pubmarker.publish(deleteMarkers("rf0"))
	target = (data.pose.position.x,data.pose.position.y,data.pose.position.z)
	pubmarker.publish(createMarker("rf0",    target))

def deleteMarkers(frame):
    global marker_id

    marker = Marker()
    marker.id = marker_id

    marker.header.frame_id = frame
    marker.type = marker.SPHERE
    marker.action = marker.DELETEALL
    marker_id += 1
    return marker
    
def createMarker(frame, position):
    global marker_id

    marker = Marker()
    marker.id = marker_id

    marker.header.frame_id = frame

    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.w = 1

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.
    marker.color.r = 1.
    marker.color.g = 0
    marker.color.b = 0
    marker_id += 1
    return marker
    
def handle_pose(msg, framename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = msg.header.stamp
    t.header.frame_id = "world"
    t.child_frame_id = framename+"_"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(t)
    
    t2 = geometry_msgs.msg.TransformStamped()
    t2.transform.translation.x = 0
    t2.transform.translation.y = 0
    t2.transform.translation.z = 0
    q = quaternion_from_euler(-pi/2, 0, 0)
    t2.transform.rotation.x = q[0]
    t2.transform.rotation.y = q[1]
    t2.transform.rotation.z = q[2]
    t2.transform.rotation.w = q[3]
    t2.child_frame_id = framename
    t2.header.frame_id = t.child_frame_id
    t2.header.stamp = msg.header.stamp
    
    br.sendTransform(t2)
    pubarea.publish(createArea("rf0"))
    if target is not None:
        pubmarker.publish(createMarker("rf0",    target))

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    rospy.Subscriber('/aruco_simple0/pose',
                     geometry_msgs.msg.PoseStamped,
                     handle_pose,
                     'rf0')
    rospy.Subscriber('/aruco_simple1/pose',
                     geometry_msgs.msg.PoseStamped,
                     handle_pose,
                     'robot')
    target_sub = rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, target_callback, queue_size=1)

    
    rospy.spin()


