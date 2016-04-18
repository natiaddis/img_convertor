#!/usr/bin/env python

import numpy as np 
import cv2, rospy
from sensor_msgs.msg import CompressedImage

'''

The images are saved in "imgs" folder in your home directory.

	capture()-> uses a window to show the webcam frames and wait for a user to click to take the image and publish
	simple_capture()-> simply takes images and publish them indefinatly until the node is killed

	--from the two capturing methods "simple_capture()" is used

'''


class ImageCapture(object):


	# initialize the image, message and publisher
	def init(self):
		self.clicked = False 
		self.img = ''
		self.msg = CompressedImage()
		self.publisher = rospy.Publisher("/compressed_image",CompressedImage)

	# the main loop
	def run(self):
		try:
			self.init()
			while not rospy.is_shutdown():
				self.simple_capture()
				self.publish_img()
				rospy.Rate(100).sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.clean()

	# close all opened image capturing windows
	def clean(self):
		cv2.destroyAllWindows()

	# to capture image in a controlled window
	# it doesn't save the image until the window is clicked
	def capture(self):
		cv2.namedWindow('image capture', cv2.WINDOW_NORMAL)
		cv2.setMouseCallback('image capture', self.onMouseClick)
		camera = cv2.VideoCapture(0)
		sucess, frame = camera.read()
		while sucess and cv2.waitKey(1) == -1 and not self.clicked:
		    cv2.imwrite('~/imgs/snapshot.png', frame)
		    cv2.imshow('image capture', frame)
		    sucess, frame = camera.read()
		cv2.imshow('image capture', frame)
		cv2.imwrite('~/imgs/snapshot.png', frame)
		self.img = frame
		print 'photo taken press any key to exit'
		cv2.waitKey()
		self.clicked = False
		cv2.destroyWindow('image capture')
		cv2.destroyAllWindows()
		del(camera) 

	# simply capture the image
	def simple_capture(self):
		camera = cv2.VideoCapture(0)
		sucess, frame = camera.read()
		self.img = frame
		cv2.imwrite('~/imgs/snapshot.png', self.img)
		del(camera) 
 
	# publish the captured image
	def publish_img(self):
		self.msg.header.stamp = rospy.Time.now()
		self.msg.format = "png"
		self.msg.data = np.array(cv2.imencode('.png',self.img)[1]).tostring()
		self.publisher.publish(self.msg)

	# mouse listener for the image capturing window
	def onMouseClick(self,event, x, y, flags, param):
		if event == cv2.cv.CV_EVENT_LBUTTONUP:
			self.clicked = True

if __name__ == '__main__':
  rospy.init_node('img_capture_node')
  img_cap = ImageCapture()
  img_cap.run()