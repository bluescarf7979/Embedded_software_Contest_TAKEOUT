#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

import numpy as np

from std_msgs.msg import Float64, String

from geometry_msgs.msg import Twist


class vel_pub :
    def __init__(self):
        rospy.init_node('platform_sub', anonymous=True)
        self.cmd_vel = rospy.Subscriber("/mobility_state", String, self.state_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.ctrl_msg = Twist()
        self.state=String()
        
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
        
            print(self.state)
            if self.state == "go":
                self.ctrl_msg.linear.x=0.8
                self.ctrl_msg.angular.z=0
            elif self.state == "slow_down":
                self.ctrl_msg.linear.x=0.3
                self.ctrl_msg.angular.z=0
            elif self.state == "stop":
                self.ctrl_msg.linear.x=0
                self.ctrl_msg.angular.z=0
            elif self.state == "back":
                self.ctrl_msg.linear.x=-0.5
                self.ctrl_msg.angular.z=0

            self.vel_pub.publish(self.ctrl_msg)
            rate.sleep()

    def state_callback(self, msg):
        self.state=msg.data
        

if __name__ == '__main__':
    try:
        test = vel_pub()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
