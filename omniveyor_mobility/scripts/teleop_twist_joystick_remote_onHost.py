#!/usr/bin/env python

"""
Utility for extracting the velocity command of this robot from a multicast topic.
Utility for publishing the localization of the robot in map frame to a multicasted topic.
"""

import rospy
import os
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as ts
import numpy as np

class multiRobotCoordinator():
    def __init__(self, nodeList):
        self.numRobots = len(nodeList)
        self.nodeList = nodeList
        self.vel_cmd_msg = []
        self.localization_sub = []
        self.robotLocationsInMap = []
        self.robotVelocities = []
        self.cmdPub = []
        for i in range(0, self.numRobots):
            self.vel_cmd_msg.append(Twist())
            self.localization_sub.append(rospy.Subscriber('robot_'+str(nodeList[i])+'/odom/filtered', 
                                        Odometry, self.globalLocCb, (nodeList[i],)))
            self.robotLocationsInMap.append([0.,0.,0.])     # Px, Py, Theta
            self.robotVelocities.append([0.,0.,0.])         # Vx, Vy, Omega
            self.cmdPub.append(rospy.Publisher('robot_'+str(nodeList[i])+'/cmd_vel', Twist, queue_size = 1))
        #self.syncPub = rospy.Publisher('/multi_robot/sync', Empty, queue_size = 1)
        print("Hooked up to topics.")

    def setTargetVels(self, vList):
        assert len(vList) == self.numRobots
        for i in range(0, self.numRobots):
            self.vel_cmd_msg[i].linear.x = vList[i][0]
            self.vel_cmd_msg[i].linear.y = vList[i][1]
            self.vel_cmd_msg[i].angular.z = vList[i][2]
            self.cmdPub[i].publish(self.vel_cmd_msg[i])

    def globalLocCb(self, msg, args):
        idx = self.nodeList.index(args[0])
        self.robotLocationsInMap[idx][0] = msg.pose.pose.position.x
        self.robotLocationsInMap[idx][1] = msg.pose.pose.position.y
        self.robotLocationsInMap[idx][2] = ts.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                    msg.pose.pose.orientation.y,
                                                                    msg.pose.pose.orientation.z,
                                                                    msg.pose.pose.orientation.w])[2]
        self.robotVelocities[idx][0] = msg.twist.twist.linear.x
        self.robotVelocities[idx][1] = msg.twist.twist.linear.y
        self.robotVelocities[idx][2] = msg.twist.twist.angular.z

class demoOne():
    def __init__(self, nodeID=0):
        self.robotIO = multiRobotCoordinator([nodeID])
        self.v_lim = [0.5, 0.5, 0.5]
        self.v_robot = [[0.,0.,0.]]
        rospy.Subscriber('joy',Joy, self.joy_cb)

    def joy_cb(self,msg):
        self.v_robot[0] = [msg.axes[1]*self.v_lim[0], msg.axes[0]*self.v_lim[1], msg.axes[2]*self.v_lim[2]]

    def run(self):
        # subscribes the joystick commands, and generates 6dof motion at the manipulator end.
        rate = rospy.Rate(60.)
        while not rospy.is_shutdown():
            self.robotIO.setTargetVels(self.v_robot)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('single_robot_cmd_loc_host')
    demo1 = demoOne()
    demo1.run()
