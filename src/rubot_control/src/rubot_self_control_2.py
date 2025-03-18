#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign

class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parameters
        self._speedFactor = rospy.get_param("~speed_factor", 1.0)
        self._distanceLaser = rospy.get_param("~distance_laser", 0.5)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)
        self.scan = None

        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

        self.closestDistance = float("inf")
        self.angleClosestDistance = float("inf")
        self.__scanRangesLengthCorrectionFactor = 2
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, scan):
        if self.scan != scan:
            self.scan = scan

    def run(self):
        while self.scan is None:
            pass
        msg = Twist()
        local_scan = None
        while not rospy.is_shutdown():
            if local_scan != self.scan:
                local_scan = self.scan
                self.__scanRangesLengthCorrectionFactor = int(len(local_scan.ranges) / 360)
                self.closestDistance, elementIndex = min(
                    (val, idx) for (idx, val) in enumerate(local_scan.ranges)
                    if local_scan.range_min < val < local_scan.range_max
                )
                self.angleClosestDistance = elementIndex / self.__scanRangesLengthCorrectionFactor
                self.angleClosestDistance = self.angleClosestDistance - 180
                # To wrapp the angle to the ranege [+180, -180]
                self.angleClosestDistance = (self.angleClosestDistance + 180) % 360 - 180
            rospy.loginfo(f"Closest distance: {self.closestDistance:.2f} meters and Angle: {self.angleClosestDistance:.1f} degrees")
            msg = Twist()
            if self.closestDistance < self._distanceLaser and -80 < self.angleClosestDistance < 80:
                msg.linear.x = self.backward_speed * self._speedFactor
                msg.angular.z = self.rotation_speed * self._speedFactor
            else:
                msg.linear.x = self.forward_speed * self._speedFactor
                msg.angular.z = 0.0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = SimpleNavigator()
    node.run()