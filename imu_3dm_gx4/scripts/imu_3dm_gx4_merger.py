#!/usr/bin/env python

# A simple ros node for merging filtered "/imu_3dm_gx4/filter/orientation" into
# "/imu_3dm_gx3/imu/orientation". No synchronization

import rospy
from imu_3dm_gx4.msg import FilterOutput
from sensor_msgs.msg import Imu


class Merger:

    def __init__(self):
        self.pub = rospy.Publisher('/imu_3dm_gx4/mergedimu', Imu,
                                   queue_size=10)
        self.filter_sub = rospy.Subscriber('/imu_3dm_gx4/filter',
                                           FilterOutput,
                                           self.filter_callback)
        self.imu_sub = rospy.Subscriber('/imu_3dm_gx4/imu', Imu,
                                        self.imu_callback)

        self.orientation = None
        self.orientation_covariance = None

    def filter_callback(self, msg):
        self.orientation = msg.orientation
        self.orientation_covariance = msg.orientation_covariance

    def imu_callback(self, msg):
        if self.orientation:
            outmsg = msg
            outmsg.orientation = self.orientation
            outmsg.orientation_covariance = self.orientation_covariance
            self.pub.publish(outmsg)

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node('imu_3dm_gx4_merger')
    merger = Merger()
    merger.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
