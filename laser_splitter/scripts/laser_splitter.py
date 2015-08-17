#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
import numpy
import copy

laser_range = (None, 1.9199)
height_range = (2.059, 2.164)
height_offset = 0.44


def laser_callback(msg, pub):
    laser_max_idx = int((laser_range[1] - msg.angle_min) / msg.angle_increment)
    assert laser_max_idx < len(msg.ranges), 'Assert: laser_max_idx(=%d) < len(msg.ranges)(=%d) failed.' % (
        laser_max_idx, len(msg.ranges))

    # extract normal laser segment
    laser_msg = copy.deepcopy(msg)
    laser_msg.angle_max = laser_range[1]
    laser_msg.ranges = msg.ranges[:laser_max_idx]
    laser_msg.intensities = msg.intensities[:laser_max_idx]

    height_idx = (int((height_range[0] - msg.angle_min) / msg.angle_increment),
                  int((height_range[1] - msg.angle_min) / msg.angle_increment))

    # extract height segment
    h = numpy.mean(msg.ranges[height_idx[0]:(height_idx[1] + 1)])
    height_msg = Vector3Stamped()
    height_msg.header = msg.header
    height_msg.vector.x = 0
    height_msg.vector.y = 0
    height_msg.vector.z = h - height_offset

    # publish messages
    pub['laser'].publish(laser_msg)
    pub['height'].publish(height_msg)

if __name__ == "__main__":
    rospy.init_node('laser_splitter')

    laser_pub = rospy.Publisher('~scan_out', LaserScan, queue_size=10)
    height_pub = rospy.Publisher('~height', Vector3Stamped, queue_size=10)

    laser_sub = rospy.Subscriber('~scan_in', LaserScan, laser_callback, callback_args=dict(
        laser=laser_pub, height=height_pub))

    rospy.spin()
