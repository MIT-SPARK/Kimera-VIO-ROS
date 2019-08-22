#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
import csv

# TODO(marcus): add support for choosing EuRoC standard or SWE format.

class GTLoggerNode:
    def __init__(self):
        self.gt_topic = rospy.get_param("~gt_topic", "/test")
        self.output_dir = rospy.get_param("~output_dir", "/home/marcus/catkin_ws/src/spark_vio_ros/output_logs/test_0/")
        self.output_csv_file = self.output_dir + "output_gt_poses.csv"

        rospy.Subscriber(self.gt_topic, TransformStamped, self.gt_cb)
        self.setup_gt_file()

    def gt_cb(self, msg):
        """ Callback for ground-truth poses.

            Writes each pose to a csv file in the swe format. This format is
            as follows:
                timestamp[ns], x, y, z, qx, qy, qz, qw, vx, vy, vz, bgx, bgy, \
                bgz, bax, bay, baz
            x, y, z are position coordinates in the world frame and qx-qw are
            quaternion values, also in the world frame. Everything afterwards
            is zero.

            Args:
                msg: A geometry_msgs/TransformStamped object representing
                     the current ground-truth body transform.
        """
        with open(self.output_csv_file, mode='a') as file:
            # Because csv only accepts one-char delimiters, we add the space
            # in each datum.
            writer = csv.writer(file, delimiter=",")
            writer.writerow([str(msg.header.stamp.to_nsec()),
                             ' ' + str(msg.transform.translation.x),
                             ' ' + str(msg.transform.translation.y),
                             ' ' + str(msg.transform.translation.z),
                             ' ' + str(msg.transform.rotation.x),
                             ' ' + str(msg.transform.rotation.y),
                             ' ' + str(msg.transform.rotation.z),
                             ' ' + str(msg.transform.rotation.w),
                             ' ' + str(0), ' ' + str(0), ' ' + str(0),
                             ' ' + str(0), ' ' + str(0), ' ' + str(0),
                             ' ' + str(0), ' ' + str(0), ' ' + str(0)])

    def setup_gt_file(self):
        """ File initializer.

            Overwrites the existing output file and writes the first row,
            which is the header (column names).
        """
        with open(self.output_csv_file, mode='wb') as file:
            writer = csv.writer(file, delimiter=",")
            writer.writerow(['timestamp[ns]', ' x', ' y', ' z', ' qx', ' qy',
                ' qz', ' qw', ' vx', ' vy', ' vz', ' bgx', ' bgy', ' bgz',
                ' bax', ' bay', ' baz'])


if __name__ == "__main__":
    rospy.init_node("gt_logger_node")
    gtln = GTLoggerNode()
    rospy.spin()
