#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion

rospy.init_node("visited_path_publisher", anonymous=True)
freq      = rospy.get_param('~freq', 5)
save_time = rospy.get_param('~save_time', 30)
pub       = rospy.Publisher("/visited_path", Path, queue_size=10)
rate      = rospy.Rate(freq)

path_msg = Path()
listener = tf.TransformListener()
poses    = []

while not rospy.is_shutdown():

  try:
    (trans,rot) = listener.lookupTransform('/map', '/cart_center', rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    continue

  # print(trans)

  pose = PoseStamped()
  pose.header.stamp    = rospy.Time.now()
  pose.header.frame_id = '/map'
  pose.pose.position.x = trans[0]
  pose.pose.position.y = trans[1]
  pose.pose.position.z = 0
  # pose.pose.orientation = tf.createQuaternionMsgFromYaw(rot[2])
  orientation = tf.transformations.quaternion_from_euler(0, 0, rot[2])
  pose.pose.orientation.x = orientation[0]
  pose.pose.orientation.y = orientation[1]
  pose.pose.orientation.z = orientation[2]
  pose.pose.orientation.w = orientation[3]

  if len(poses) > freq*save_time:
    poses.pop(0)
  poses.append(pose)

  path_msg.header = pose.header
  path_msg.poses = poses

  # print("Publish visited path")

  pub.publish(path_msg)
  rate.sleep()
