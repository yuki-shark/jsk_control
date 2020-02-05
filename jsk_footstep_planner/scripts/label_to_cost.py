#!/usr/bin/env python
# from __future__ import print_function

# import roslib
# roslib.load_manifest('safe_footstep_planner')
import sys
import os
import rospy
import rospkg
import json
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class label_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/object_cost_image",Image)
    self.labels_pub = rospy.Publisher("/known_labels", Int8MultiArray)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/object_label_image", Image, self.callback)
    rospack = rospkg.RosPack()
    self.path = rospy.get_param('~data_base_path', os.path.join(rospack.get_path('jsk_footstep_planner'), 'config/database.json'))

  def getDatabase(self):
    fn = open(self.path, 'r')
    database_raw = json.load(fn)
    # database = dict([(0,0), (2, 50), (3, 254)])
    database = dict()
    for key in database_raw.keys():
        database[int(key)] = database_raw[key]
    return database

  def pubLabelMsg(self, known_labels):
    msg = Int8MultiArray()
    dim = MultiArrayDimension()
    dim.label  = "length"
    dim.size   = len(known_labels)
    dim.stride = len(known_labels)
    msg.layout.dim.append(dim)
    msg.layout.data_offset = 0
    msg.data = known_labels
    self.labels_pub.publish(msg)

  def callback(self, data):
    try:
      label_image = self.bridge.imgmsg_to_cv2(data, "mono8")
      # label_image = self.bridge.imgmsg_to_cv2(data, "32SC1")
    except CvBridgeError as e:
      print(e)

    label_image = np.array(label_image).astype(np.int64)
    shape = label_image.shape
    database = self.getDatabase()
    cost_image = np.vectorize(database.get)(label_image, 0)

    # convert type to "32SC1"
    # cost_image = cost_image.astype(np.int32)
    # convert type to "MONO8"
    cost_image = cost_image.astype(np.uint8)

    try:
      # image_msg = self.bridge.cv2_to_imgmsg(cost_image, "32SC1")
      image_msg = self.bridge.cv2_to_imgmsg(cost_image, "mono8")
      image_msg.header = data.header
      self.image_pub.publish(image_msg)
    except CvBridgeError as e:
      print(e)

    known_labels = list(map(lambda x: np.uint8(x), database.keys()))
    self.pubLabelMsg(known_labels)

def main(args):
  rospy.init_node('label_converter', anonymous=True)
  lc = label_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
