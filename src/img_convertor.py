#!/usr/bin/env python

import numpy as np 
import cv2, rospy
from sensor_msgs.msg import CompressedImage

class ImageConvertor(object):

	# initialize the image, message, publisher and subscriber
	# subscribes to a "compressed_image" topic
	def init(self):
		self.img = ''
		self.msg = CompressedImage()
		self.subscriber = rospy.Subscriber("/compressed_image",CompressedImage, self.callback, queue_size = 10)
		self.publisher = rospy.Publisher("/converted_image",CompressedImage)

	# main loop
	def run(self):
		self.init()
		rospy.spin()

	# recieves the commpressed image change the color to gray and publish
	def callback(self,ros_data):
		np_arr = np.fromstring(ros_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		gray = cv2.cvtColor( image_np, cv2.COLOR_BGR2GRAY )
		self.img = gray
		cv2.imwrite('/home/natnael/imgs/recieved.png',image_np)
		cv2.imwrite('/home/natnael/imgs/gray.png',self.img)
		self.publish_img()

	# publishes a "converted_image" topic
	def publish_img(self):
		self.msg.header.stamp = rospy.Time.now()
		self.msg.format = "png"
		self.msg.data = np.array(cv2.imencode('.png',self.img)[1]).tostring()
		self.publisher.publish(self.msg)


if __name__ == '__main__':
  rospy.init_node('img_convertor_node')
  img_con = ImageConvertor()
  img_con.run()