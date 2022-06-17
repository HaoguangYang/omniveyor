#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np

class TrajWriter():
    def __init__(self):
        
        self.traj = []
        rospy.Subscriber('/move_base_simple/goal',PoseStamped, self.goal_cb)
        self.fname = '/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/traj/test_traj.txt'
        rospy.on_shutdown(self.on_kill)

    def goal_cb(self,msg):
        """_summary_

        Args:
            msg (_type_): a goal waypoint for to be added to the trajectory
        """        
        self.traj.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
 msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    
    def main(self):
        rospy.spin()

    def on_kill(self):
        np.savetxt(self.fname, np.array(self.traj), delimiter=' ')

if __name__=="__main__":
    rospy.init_node('trajectory_writer', anonymous=True)
    tw = TrajWriter()
    tw.main()

