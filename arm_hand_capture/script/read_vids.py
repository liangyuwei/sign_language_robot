#!/usr/bin/env python

import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError # convert OpenCV images to ROS image messages
from sensor_msgs.msg import Image

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("test_read_image",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("test_read_image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)



if __name__ == '__main__':

  ### Initialize a ROS node for publishing image messages
  rospy.init_node('image_converter', anonymous=True)


  ### Parameters
  width = 1280 #640  # default is 640
  height = 720 #480  # default is 480
  # problem with using YUYV: still, the size of the image would affect the FPS.. requesting larger size for images from camera would reduce FPS, unless you do post-processing yourself...

  ### Read image from camera driver
  cap = cv2.VideoCapture(0)
  cap.set(3, width)
  cap.set(4, height) # set desired width and height
  #cap.set(cv2.CAP_PROP_FPS, 30) # cannot exceed 30 fps due to hardware limit
  cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

  if not cap.isOpened():
      raise IOError("Cannot open webcam!")
  
  import pdb
  start = rospy.get_rostime()
  count = 1.0
  while(True):
    # capture frame-by-frame
    ret, frame = cap.read() # (480, 640, 3)
  
    # get information 
    wid = cap.get(3)
    hei = cap.get(4)
    fps = cap.get(cv2.CAP_PROP_FPS)
    #
    now = rospy.get_rostime()
    afps = count / (now-start).to_sec()
    count = count + 1
    print('Width={}, Height={}, FPS={}, actual FPS={}'.format(wid, hei, fps, afps))
    # actual FPS is still around 14 Hz... damn... limited by hardware, can't help

    # operations on the frames received
    frame = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)

    # display 
    cv2.imshow('Input', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  
  # clean up
  cap.release()
  cv2.destroyAllWindows()

                
  ### Convert OpenCV image to ROS image message
#  bridge = CvBridge()
#  imgmsg = bridge.cv2_to_imgmsg(, encoding='passthrough')
#  import pdb
#  pdb.set_trace()


  ### Publish ROS image message

    







