#!/usr/bin/env python
import os
import csv
import rospy
from sensor_msgs.msg import Imu

class DeltasLoggerNode:
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir")
        self.imu_topic = rospy.get_param("~imu_topic")
        self.imu_output_csv_file = os.path.join(self.output_dir, "timestamps_imu.csv")
        self.previous_imu = None

        rospy.Subscriber(self.imu_topic, Imu, self.record_imu_msg, queue_size=20)
        self.setup_imu_file()

    def record_imu_msg(self, msg):
        """ Callback for IMU data as it comes in as Imu msgs.

            Writes each point to a csv file in the swe format. This format is
            as follows:
                timestamp[ns], delta time

            Args:
                msg: A sensor_msgs/Imu object representing
                     the current Imu sensor recording.
        """
        with open(self.imu_output_csv_file, mode='a') as file:
            # Because csv only accepts one-char delimiters, we add the space
            # in each datum.
            if (self.previous_imu):
                delta = msg.header.stamp.to_nsec() - self.previous_imu
                if (delta < 0):
                    delta = delta*100
                writer = csv.writer(file, delimiter=",")
                writer.writerow([str(msg.header.stamp.to_nsec()),str(delta)])

            self.previous_imu = msg.header.stamp.to_nsec()


    def setup_imu_file(self):
        """ File initializer.

            Overwrites the existing output file and writes the first row,
            which is the header (column names).
        """
        with open(self.imu_output_csv_file, mode='wb') as file:
            writer = csv.writer(file, delimiter=",")
            writer.writerow(['timestamp', 'delta'])

if __name__ == "__main__":
    rospy.init_node("timestamps_logger_node")
    imuln = DeltasLoggerNode()
    rospy.spin()
