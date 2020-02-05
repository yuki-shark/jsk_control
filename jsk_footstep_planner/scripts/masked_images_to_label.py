#!/usr/bin/env python
# from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/object_label_image",Image)

    self.bridge = CvBridge()

    # self.data_red_sub    = message_filters.Subscriber("/red_color_filter/image", Image)
    self.data_green_sub  = message_filters.Subscriber("/green_hsv_color_filter/image", Image)
    self.data_blue_sub   = message_filters.Subscriber("/blue_hsv_color_filter/image", Image)
    # self.data_yellow_sub = message_filters.Subscriber("/yellow_color_filter/image", Image)

    # self.ts = message_filters.ApproximateTimeSynchronizer([self.data_red_sub, self.data_green_sub, self.data_blue_sub, self.data_yellow_sub], 10, 0.3)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.data_green_sub, self.data_blue_sub], 10, 0.3)

    self.ts.registerCallback(self.callback)
    # self.red_label    = rospy.get_param('~red_label', 1)
    self.green_label  = rospy.get_param('~green_label', 2)
    self.blue_label   = rospy.get_param('~blue_label', 3)
    # self.yellow_label = rospy.get_param('~yellow_label', 4)

  def callback(self, data_green, data_blue):
    try:
      # cv_image_red = self.bridge.imgmsg_to_cv2(data_red, "mono8")
      cv_image_green = self.bridge.imgmsg_to_cv2(data_green, "mono8")
      cv_image_blue = self.bridge.imgmsg_to_cv2(data_blue, "mono8")
      # cv_image_yellow = self.bridge.imgmsg_to_cv2(data_yellow, "mono8")
    except CvBridgeError as e:
      print(e)

    label_image = np.zeros(cv_image_green.shape)

    # set labels as num
    # label_image += cv_image_red    * 1.0/255 * self.red_label     # Label red
    label_image += cv_image_green  * 1.0/255 * self.green_label   # Label green
    label_image += cv_image_blue   * 1.0/255 * self.blue_label    # Label blue
    # label_image += cv_image_yellow * 1.0/255 * self.yellow_label  # Label yellow

    # convert type to "32SC1"
    # label_image = label_image.astype(np.int32)
    # convert type to "MONO8"
    label_image = label_image.astype(np.uint8)

    try:
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(label_image, "32SC1"))
      label_msg = self.bridge.cv2_to_imgmsg(label_image, "mono8")
    except CvBridgeError as e:
      print(e)

    label_msg.header = data_green.header
    self.image_pub.publish(label_msg)

def main(args):
  rospy.init_node('masked_images_to_label', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
