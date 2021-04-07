#!/usr/bin/env python

import os
import csv

import rospy
from sensor_msgs.msg import Imu

# TODO(marcus): add support for choosing EuRoC standard or SWE format.
class IMULoggerNode:
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic")
        self.output_dir = rospy.get_param("~output_dir")
        self.output_csv_file = os.path.join(self.output_dir, "imu_data.csv")

        # rospy.Subscriber(self.gt_topic, TransformStamped, self.gt_cb_tfs)
        rospy.Subscriber(self.imu_topic, Imu, self.record_imu_msg, queue_size=20)
        self.setup_imu_file()

    def record_imu_msg(self, msg):
        """ Callback for IMU data as it comes in as Imu msgs.

            Writes each point to a csv file in the swe format. This format is
            as follows:
                timestamp[ns], ang_vel_x, ang_vel_y, ang_vel_z, 
                lin_acc_x, lin_acc_y, lin_acc_z

            Args:
                msg: A sensor_msgs/Imu object representing
                     the current Imu sensor recording.
        """
        with open(self.output_csv_file, mode='a') as file:
            # Because csv only accepts one-char delimiters, we add the space
            # in each datum.
            writer = csv.writer(file, delimiter=",")
            writer.writerow([str(msg.header.stamp.to_nsec()),
                             str(msg.angular_velocity.x),
                             str(msg.angular_velocity.y),
                             str(msg.angular_velocity.z),
                             str(msg.linear_acceleration.x),
                             str(msg.linear_acceleration.y),
                             str(msg.linear_acceleration.z)])

    def setup_imu_file(self):
        """ File initializer.

            Overwrites the existing output file and writes the first row,
            which is the header (column names).
        """
        with open(self.output_csv_file, mode='wb') as file:
            writer = csv.writer(file, delimiter=",")
            writer.writerow(['timestamp[ns]', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z', 
            'lin_acc_x', 'lin_acc_y', 'lin_acc_z'])

if __name__ == "__main__":
    rospy.init_node("imu_logger_node")
    imuln = IMULoggerNode()
    rospy.spin()
