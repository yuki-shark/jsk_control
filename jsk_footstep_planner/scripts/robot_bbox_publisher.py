#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty as EmptySrv
from jsk_recognition_msgs.msg import BoundingBox
from jsk_footstep_msgs.msg import FootstepArray
from geometry_msgs.msg import Point, Quaternion, Vector3

rospy.init_node("robot_bbox_publisher", anonymous=True)
p = rospy.Publisher("/robot_bbox", BoundingBox, queue_size=10)
r = rospy.Rate(100)
transport_object = rospy.get_param('~transport_object', "push_cart")

rospy.loginfo("Clearing /accumulated_heightmap/reset due to change of robot_bbox")
srv = rospy.ServiceProxy("/accumulated_heightmap/reset", EmptySrv)
try:
  srv()
except:
  rospy.logerr("Failed to reset /accumulated_heightmap")

bbox_msg = BoundingBox()
bbox_msg.header.frame_id = 'body_on_odom'

if transport_object == "push_cart":
  position = Point(0.75, 0.0, 1.25)
  orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
  dimensions = Vector3(1.5, 1.2, 3.0)
elif transport_object == "wheelbarrow":
  position = Point(0.9, 0.0, 1.25)
  orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
  dimensions = Vector3(2.0, 1.2, 3.0)
else:
  position = Point(0.0, 0.0, 1.25)
  orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
  dimensions = Vector3(0.5, 1.0, 3.0)

bbox_msg.pose.position = position
bbox_msg.pose.orientation = orientation
bbox_msg.dimensions = dimensions

while not rospy.is_shutdown():
  p.publish(bbox_msg)
  r.sleep()
