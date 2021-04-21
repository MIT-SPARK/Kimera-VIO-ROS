#!/usr/bin/env python
import os
import csv
import rospy
from sensor_msgs.msg import Image

class DeltasLoggerNode:
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir")
        self.image_topic = rospy.get_param("~right_cam_topic")
        self.image_output_csv_file = os.path.join(self.output_dir, "timestamps_infra2.csv")

        rospy.Subscriber(self.image_topic, Image, self.record_image_msg, queue_size=20)
        self.setup_image_file()

    def record_image_msg(self, msg):
        """ Callback for Image data as it comes in as Image msgs.

            Writes each point to a csv file in the swe format. This format is
            as follows:
                timestamp[ns]

            Args:
                msg: A sensor_msgs/Image object representing
                     the current infra Image recording.
        """
        with open(self.image_output_csv_file, mode='a') as file:
            # Because csv only accepts one-char delimiters, we add the space
            # in each datum.
            writer = csv.writer(file, delimiter=",")
            writer.writerow([str(msg.header.stamp.to_nsec())])



    def setup_image_file(self):
        """ File initializer.

            Overwrites the existing output file and writes the first row,
            which is the header (column names).
        """
        with open(self.image_output_csv_file, mode='wb') as file:
            writer = csv.writer(file, delimiter=",")
            writer.writerow(['timestamp'])

if __name__ == "__main__":
    rospy.init_node("infra2_logger_node")
    imageln = DeltasLoggerNode()
    rospy.spin()
