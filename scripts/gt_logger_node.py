#!/usr/bin/env python

import os
import csv

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# TODO(marcus): add support for choosing EuRoC standard or SWE format.
class GTLoggerNode:
    def __init__(self):
        self.gt_topic = rospy.get_param("~gt_topic")
        self.output_dir = rospy.get_param("~output_dir")
        self.output_csv_file = os.path.join(self.output_dir, "traj_gt.csv")

        # rospy.Subscriber(self.gt_topic, TransformStamped, self.gt_cb_tfs)
        rospy.Subscriber(self.gt_topic, Odometry, self.gt_cb_odom, queue_size=20)
        self.setup_gt_file()

    def gt_cb_tfs(self, msg):
        """ Callback for ground-truth poses as they come in as TransformStamped
            msgs.

            Writes each pose to a csv file in the swe format. This format is
            as follows:
                timestamp[ns], x, y, z, qw, qx, qy, qz
            x, y, z are position coordinates in the world frame and qx-qw are
            quaternion values, also in the world frame. Everything afterwards
            is zero.

            Args:
                // TODO: this should be a odometry msg...
                msg: A geometry_msgs/TransformStamped object representing
                     the current ground-truth body transform.
        """
        with open(self.output_csv_file, mode='a') as file:
            # Because csv only accepts one-char delimiters, we add the space
            # in each datum.
            writer = csv.writer(file, delimiter=",")
            writer.writerow([str(msg.header.stamp.to_nsec()),
                             str(msg.transform.translation.x),
                             str(msg.transform.translation.y),
                             str(msg.transform.translation.z),
                             str(msg.transform.rotation.w),
                             str(msg.transform.rotation.x),
                             str(msg.transform.rotation.y),
                             str(msg.transform.rotation.z)])

    def gt_cb_odom(self, msg):
        """ Callback for ground-truth poses as they come in as Odometry msgs.

            Writes each pose to a csv file in the swe format. This format is
            as follows:
                timestamp[ns], x, y, z, qw, qx, qy, qz
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
                             str(msg.pose.pose.position.x),
                             str(msg.pose.pose.position.y),
                             str(msg.pose.pose.position.z),
                             str(msg.pose.pose.orientation.w),
                             str(msg.pose.pose.orientation.x),
                             str(msg.pose.pose.orientation.y),
                             str(msg.pose.pose.orientation.z)])

    def setup_gt_file(self):
        """ File initializer.

            Overwrites the existing output file and writes the first row,
            which is the header (column names).
        """
        with open(self.output_csv_file, mode='wb') as file:
            writer = csv.writer(file, delimiter=",")
            writer.writerow(['timestamp[ns]', 'x', 'y', 'z', 'qw', 'qx',
                             'qy', 'qz'])


if __name__ == "__main__":
    rospy.init_node("gt_logger_node")
    gtln = GTLoggerNode()
    rospy.spin()
