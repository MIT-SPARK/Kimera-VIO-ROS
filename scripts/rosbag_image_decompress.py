import rosbag
import rospy
import sys
import numpy as np
import sensor_msgs.msg as sensor_msg
import cv2
from cv_bridge import CvBridge

def process_bag(input_rosbag, output_rosbag, compressed_image_topics=[], output_image_topics=[]):
	if (len(compressed_image_topics) > len(output_image_topics)):
		print("Expect an output image topic for each compressed image topic. ")
		return

	bridge = CvBridge()
	for topic, msg, t in input_rosbag.read_messages():
		try:
			# Decompress Image 
			if topic in compressed_image_topics:
				np_img = np.fromstring(msg.data, np.uint8)
				cv_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
				grayscale = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
				img_msg = bridge.cv2_to_imgmsg(grayscale, "mono8")
				img_msg.header = msg.header
				output_bag.write(output_image_topics[compressed_image_topics.index(topic)], img_msg)
			elif topic in output_image_topics:
				pass
			else:
				output_bag.write(topic, msg)
		except AttributeError: 
			output_bag.write(topic, msg)
	return

if __name__=="__main__":
	if len(sys.argv) != 5:
		print("Example Usage: python rosbag_image_decompress.py input.bag output.bag /cam/infra1/image_rect_raw/compressed,/cam/infra2/image_rect_raw/compressed /cam/infra1/image_rect_raw,/cam/infra2/image_rect_raw")
		print("Note that the topics are comma separated with no space. ")
	input_bag = rosbag.Bag(sys.argv[1])
	output_bag = rosbag.Bag(sys.argv[2], 'w')
	compressed_img_topics = sys.argv[3].split(",")
	decompressed_img_topics = sys.argv[4].split(",")

	process_bag(input_bag, output_bag, compressed_img_topics, decompressed_img_topics)

	input_bag.close()
	output_bag.close()