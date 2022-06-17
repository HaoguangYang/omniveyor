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

class demoTrajs():
    def __init__(self, nodeList):
        self.robotIO = multiRobotCoordinator(nodeList)
        self.v_lim = [0.3, 0.3, 0.1]      # linear, angular, along rod
        self.vel_center =       np.array([0., 0., 0., 0., 0.])
        self.vel_center_des =   np.array([0., 0., 0., 0., 0.])
        self.pos_center =       np.array([0., 0., 0., 0., 0.])  # [x, y, d1, d2, d3]
        self.pos_center_des =   np.array([0., 0., 0., 0., 0.])
        self.orientation_center = 0.
        self.orientation_center_des = 0.
        self.omega = 0.
        self.omega_des = 0.
        self.robot_orientation = np.array([0., 2.*np.pi/3., -2.*np.pi/3.])
        self.v_robots =         np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
        
        self.kpl = 1.5
        self.kdl = 1.0
        self.kpa = 1.5
        self.kda = 1.0
        
        self.jacobian = np.array([[1., 0., 0., 0., 0.],
                                  [0., 1., -1., 0., 0.],
                                  [1., 0., 0., np.sqrt(3.)*0.5, 0.],
                                  [0., 1., 0., 0.5, 0.],
                                  [1., 0., 0., 0., -np.sqrt(3.)*0.5],
                                  [0., 1., 0., 0., 0.5]])
        rospy.Subscriber('joy',Joy, self.joy_cb)

    def transformVel(self):
        # transform the velocities from the center of the Y rails to the robots
        el = self.pos_center_des - self.pos_center
        eld = self.vel_center_des - self.vel_center
        ea = np.mod(self.orientation_center_des - self.orientation_center + np.pi*3., np.pi*2.)-np.pi
        ead = self.omega_des - self.omega
        ul = self.kpl*el + self.kdl*eld
        ua = self.kpa*ea + self.kda*ead

        print(el)
        print(ea)
        
        ul_glob = np.dot(self.jacobian, ul)                 # [vx1 vy1 vx2 vy2 vx3 vy3]
        ul_glob += np.array([ua*self.pos_center[2]*np.cos(self.orientation_center),
                             ua*self.pos_center[2]*np.sin(self.orientation_center),
                             ua*self.pos_center[3]*np.cos(self.orientation_center+2.*np.pi/3.),
                             ua*self.pos_center[3]*np.sin(self.orientation_center+2.*np.pi/3.),
                             ua*self.pos_center[4]*np.cos(self.orientation_center-2.*np.pi/3.),
                             ua*self.pos_center[4]*np.sin(self.orientation_center-2.*np.pi/3.)])
        self.v_robots[0,0] = ul_glob[0]*np.cos(self.robot_orientation[0]) + ul_glob[1]*np.sin(self.robot_orientation[0])
        self.v_robots[0,1] = -ul_glob[0]*np.sin(self.robot_orientation[0]) + ul_glob[1]*np.cos(self.robot_orientation[0])
        self.v_robots[1,0] = ul_glob[2]*np.cos(self.robot_orientation[1]) + ul_glob[3]*np.sin(self.robot_orientation[1])
        self.v_robots[1,1] = -ul_glob[2]*np.sin(self.robot_orientation[1]) + ul_glob[3]*np.cos(self.robot_orientation[1])
        self.v_robots[2,0] = ul_glob[4]*np.cos(self.robot_orientation[2]) + ul_glob[5]*np.sin(self.robot_orientation[2])
        self.v_robots[2,1] = -ul_glob[4]*np.sin(self.robot_orientation[2]) + ul_glob[5]*np.cos(self.robot_orientation[2])
        print(self.robot_orientation)
        self.v_robots[0,2] = self.kpa*(np.mod(self.orientation_center_des - self.robot_orientation[0] + np.pi*3., np.pi*2.)-np.pi) + self.kda*ead
        self.v_robots[1,2] = self.kpa*(np.mod(self.orientation_center_des + 2.*np.pi/3. -self.robot_orientation[1] + np.pi*3., np.pi*2.)-np.pi) + self.kda*ead
        self.v_robots[2,2] = self.kpa*(np.mod(self.orientation_center_des - 2.*np.pi/3. -self.robot_orientation[2] + np.pi*3., np.pi*2.)-np.pi) + self.kda*ead
        
        """
        self.v_robots[:,2] = self.vel_center[2]
        self.v_robots[0,0:2] = self.vel_center[0:2]
        self.v_robots[1,0:2] = [-self.vel_center[0]*0.5+self.vel_center[1]*np.sqrt(3.)*0.5,
                                -self.vel_center[0]*np.sqrt(3.)*0.5-self.vel_center[1]*0.5]
        self.v_robots[2,0:2] = [-self.vel_center[0]*0.5-self.vel_center[1]*np.sqrt(3.)*0.5,
                                self.vel_center[0]*np.sqrt(3.)*0.5-self.vel_center[1]*0.5]
        self.v_robots[0, 0] += -self.vel_center[2]*self.dist_from_center[0]
        self.v_robots[1, 0] += -self.vel_center[2]*self.dist_from_center[1]
        self.v_robots[2, 0] += -self.vel_center[2]*self.dist_from_center[2]
        #self.v_robots[:,0:2] *= -1. # for current setup which is rotated 180 degs
        """
        
    def updateGeometry(self):
        # estimates and updates how far the robots are from the center of Y rails.
        # robotIO.robotLocationsInMap...
        pose_array = np.array(self.robotIO.robotLocationsInMap)
        self.robot_orientation = pose_array[:,2]
        orientation_unwrapped = pose_array[:,2]
        # make monotonic increase
        if orientation_unwrapped[0] > orientation_unwrapped[1]:
            orientation_unwrapped[1] += (np.pi+np.pi)
        if orientation_unwrapped[0] < orientation_unwrapped[2]:
            orientation_unwrapped[2] -= (np.pi+np.pi)
        # saturate and subtract
        self.orientation_center = np.mod(np.average(orientation_unwrapped) + 5.*np.pi, np.pi*2.) - np.pi
        #print(orientation_unwrapped)
        self.jacobian[0,2] = np.sin(self.orientation_center)
        self.jacobian[1,2] = -np.cos(self.orientation_center)
        self.jacobian[2,3] = np.sin(self.orientation_center+2.*np.pi/3.)
        self.jacobian[3,3] = -np.cos(self.orientation_center+2.*np.pi/3.)
        self.jacobian[4,4] = np.sin(self.orientation_center-2.*np.pi/3.)
        self.jacobian[5,4] = -np.cos(self.orientation_center-2.*np.pi/3.)
        jacobian_pinv = np.linalg.pinv(self.jacobian)
        self.pos_center = np.dot(jacobian_pinv,
                            np.array([pose_array[0,0], pose_array[0,1], 
                                      pose_array[1,0], pose_array[1,1], 
                                      pose_array[2,0], pose_array[2,1]]))           # [x, y, d1, d2, d3]
        print(self.pos_center)
        self.orientation_center = np.arctan2(-self.pos_center[0]+pose_array[0,0], self.pos_center[1]-pose_array[0,1])
        vel_array = np.array(self.robotIO.robotVelocities)
        self.omega = (vel_array[0,2] + vel_array[1,2] + vel_array[2,2])/3.
        self.vel_center = np.dot(jacobian_pinv,
                            np.array([vel_array[0,0]*np.cos(pose_array[0,2])-vel_array[0,1]*np.sin(pose_array[0,2]),
                                      vel_array[0,0]*np.sin(pose_array[0,2])+vel_array[0,1]*np.cos(pose_array[0,2]), 
                                      vel_array[1,0]*np.cos(pose_array[1,2])-vel_array[1,1]*np.sin(pose_array[1,2]),
                                      vel_array[1,0]*np.sin(pose_array[1,2])+vel_array[1,1]*np.cos(pose_array[1,2]), 
                                      vel_array[2,0]*np.cos(pose_array[2,2])-vel_array[2,1]*np.sin(pose_array[2,2]),
                                      vel_array[2,0]*np.sin(pose_array[2,2])+vel_array[2,1]*np.cos(pose_array[2,2])])) # [x, y, d1, d2, d3]
        
    def joy_cb(self,msg):
        d1_vel = (-msg.axes[4]-msg.axes[3])*self.v_lim[2]
        d2_vel = (msg.axes[4]*0.5 + msg.axes[5]*np.sqrt(3)*0.5-msg.axes[3])*self.v_lim[2]
        d3_vel = (msg.axes[4]*0.5 - msg.axes[5]*np.sqrt(3)*0.5-msg.axes[3])*self.v_lim[2]
        self.vel_center_des = np.array([msg.axes[1]*self.v_lim[0], msg.axes[0]*self.v_lim[0], d1_vel, d2_vel, d3_vel])
        self.omega_des = msg.axes[2]*self.v_lim[1]

    def run(self):
        # subscribes the joystick commands, and generates 6dof motion at the manipulator end.
        rate_hz = 60.
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.updateGeometry()
            print(self.orientation_center)
            self.pos_center_des = self.pos_center
            self.orientation_center_des = self.orientation_center
            if (self.pos_center_des[0] != 0. and self.pos_center_des[1] != 0. and 
                abs(self.pos_center_des[2]) > 0.2 and abs(self.pos_center_des[3]) > 0.2 and abs(self.pos_center_des[4]) > 0.2):
                break
        while not rospy.is_shutdown():
            self.updateGeometry()
            self.pos_center_des = self.pos_center + self.vel_center_des/rate_hz
            self.orientation_center_des = np.mod(self.orientation_center + self.omega_des/rate_hz +np.pi, np.pi*2) - np.pi
            self.transformVel()
            print(self.v_robots)
            self.robotIO.setTargetVels(self.v_robots.tolist())
            rate.sleep()
        self.robotIO.setTargetVels([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])

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
    rospy.init_node('multi_robot_cmd_loc_host')
    demo = demoTrajs([rospy.get_param('~/node_1',   9),
                        rospy.get_param('~/node_2', 6),
                        rospy.get_param('~/node_3', 8)])     # Robot 1, 2, 3. counter-clockwise direction
    demo.run()
    #demo1 = demoOne(6)
    #demo1.run()
