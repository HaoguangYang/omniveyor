#!/usr/bin/python

import rospy
import os
import sys
import copy
import numpy as np
import threading
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from ReFrESH_ROS import ReFrESH_Module, Manager
from ReFrESH_ROS_utils import Ftype, ROSnodeMonitor
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseGoal
from omniveyor_common.msg import LowLevelNavigationGoal, LowLevelNavigationFeedback, LowLevelNavigationResult
from actionlib_msgs.msg import GoalStatus
import utils

"""multilevel inherited class to manage movebase global and local planners"""
class MoveBaseManager(ReFrESH_Module, Manager):
    def __init__(self, launcher, managedModules=[], name="moveBaseManager", priority=97, preemptive=True,
                freq=5.0, minReconfigInterval = 1.0):
        ReFrESH_Module.__init__(self, name, priority, preemptive, EX_thread=3, EV_thread=1, ES_thread=1)
        Manager.__init__(self, launcher, managedModules, name, freq, minReconfigInterval)
        self.EX[0] = self.Decider
        self.setComponentProperties('EX', Ftype.LAUNCH_FILE, 'omniveyor_mobility', 'navigation.launch', ind=1)
        self.setComponentProperties('EX', Ftype.THREAD, exec=self.goalRunner, ind=2)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 5.0})
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)
        # Resource metric: CPU time, memory
        self.resourceMetrics = [0.5, 0.0]
        # Performance metric: metric bottleneck of submodules. This metric is cleared once the goal pose change.
        # The performance metric is a member of the manager class.
        self.cpuQuota = 0.5
        self.memQuota = 0.5
        self.exMon = ROSnodeMonitor()
        # The class instance is mutable (call by reference)
        self.currentGoal = LowLevelNavigationGoal()     # goal to be submitted / currently running
        self.moveBaseGoal = MoveBaseGoal()              # goal translated to movebase format
        self.goalTolerance = [0., 0.]                   # extracted goal positioning tolerance (linear, angular)
        self.abortedGoal = None                         # last goal that returned abortion status by all managed modules
        self.abortedPose = None                         # last world pose where all managed modules returned abortion status

    """Use move base planners to reach the goal."""    
    def goalRunner(self):
        try:
            while not rospy.is_shutdown():
                # stuck here to wait for new goal to arrive
                self.managerHandle.newGoalEvent.wait()
                self.managerHandle.newGoalEvent.clear()
                # assemble goal movebase action
                self.updateGoal()
                # sync the goal to all modules
                for m in self.moduleDict:
                    m.syncGoal()
                if not len(self.onDict):
                    # nothing is on.
                    while not rospy.is_shutdown() and not self.managerHandle.newGoalEvent.isSet():
                        # sequentially query the estimators for initial capability assessment
                        # this process is identical to what the decider do when a module falls below the bar.
                        candidate = None
                        bestPerf = 1.
                        for m, EShandle in self.offDict:
                            if callable(EShandle):
                                EShandle()
                            tmp = m.reconfigMetric.bottleNeck()
                            if tmp < bestPerf:
                                candidate = m
                                bestPerf = tmp
                        # turn on the most capable planner module. This also submits the goal.
                        if not(candidate is None):
                            self.turnOn(candidate)
                            break
                        elif self.currentGoal.expiration <= rospy.Time.now():
                            # in case intermittent blockage that stops the robot, the loop constantly checks if
                            # a global plan is available. The time consumed counts towards the timeout, of course.
                            # timeout
                            self.setResult(GoalStatus.ABORTED, final=True)
                            break
                        time.sleep(0.1)
                # don't care about a goal once submitted, since the EV handles rest of the checking job.
        except SystemExit:
            #self.cancelAll()
            self.turnOffAll()

    def evaluator(self, event):
        # check if performance monitor is attached
        if self.exMon.isAttached():
            # log worst case CPU usage of the launched exec.
            uCPU, uMem = self.exMon.getCpuMemUtil()
            uCPU /= self.cpuQuota
            uMem /= self.memQuota
            # exponential filter for worst case execution time / memory utilization
            self.resourceMetrics[0] = max(uCPU, self.resourceMetrics[0]*0.975)
            self.resourceMetrics[1] = max(uMem, self.resourceMetrics[1]*0.975)
            # report self.bottleNeck to the performance aspect of the module
            # bottleneck is already updated in the decider.
            self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
            #print("MB-EV ", self.resourceMetrics)
        else:
            # attach resource monitor to the move base instance
            self.exMon.attach(self.getMyEXhandle()[1])

    def estimator(self):
        self.updateGoal()
        # movebase is now off. obtain a-priori esitmates
        if self.bottleNeck >= 1.0:
            # we had a failed record before
            distance = np.array(self.abortedGoal.target_pose.pose.position) - np.array(self.currentGoal.target_pose.pose.position)
            goalPosTol = self.goalTolerance[0]
            # the base can rotate in place, so goal pose is less important in determining success.
            if np.dot(distance, distance) > 9.*goalPosTol*goalPosTol:
                # we are receiving a goal at a new location. Reset the estimators
                self.bottleNeck = 0.0
                self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
        else:
            # this module succeeded last time. Keep using it.
            self.bottleNeck = 0.0
            self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
    
    def updateGoal(self):
        # make a local copy 
        self.currentGoal = copy.deepcopy(self.managerHandle.currentActionGoal)
        self.moveBaseGoal.target_pose.header = self.currentGoal.header
        self.moveBaseGoal.target_pose.pose = self.currentGoal.target_pose.pose
        self.goalTolerance = [0.5*(self.abortedGoal.target_pose.pose.covariance[0]+self.abortedGoal.target_pose.pose.covariance[7]),
                                self.abortedGoal.target_pose.pose.covariance[35]]
    
    def setFeedback(self, feedback):
        self.managerHandle.currentActionFeedback = feedback
    
    def setResult(self, status, final=False):
        # synchronize whatever comes in.
        self.managerHandle.currentActionStatus = status
        if final:
            # this module can not get the robot to the goal pose. ring the bell
            if (status == GoalStatus.ABORTED or status == GoalStatus.REJECTED) and self.bottleNeck >= 1.0:
                # 4: aborted, 5: rejected. bn>1: has triggered performance crisis among all available planners
                self.abortedGoal = self.currentGoal
            # the goal runner of the manager will now check the result message.
            self.managerHandle.goalDoneEvent.set()

class MotionManager(Manager):
    def __init__(self, launcher, managedModules=[], name="", freq=5.0, minReconfigInterval = 1.0,
                 worldFrame='map', odomFrame='odom', robotFrame='base_link', worldPoseTopic='world_pose'):
        super().__init__(launcher, managedModules, name, freq, minReconfigInterval)
        # handle was previously installed to super(). Need to migrate to self for the additional functions.
        for m in managedModules:
            m.managerHandle = self
        # utilities that interpolates robot's pose in the world (map frame, with fallback to odom frame)
        # tf - transform odom to map frame for amcl_pose, filtered odom,
        # fallback for other map poses published through tf.
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # Heartbeat - raw odometry is the ultimate fallback as long as the base is running.
        self.odomSub = rospy.Subscriber('/odom', Odometry, self.wheelOdomCb)
        # One level up, if the filter is running.
        self.filteredOdomTimeout = 0.1
        self.mapPoseTimeout = 3.0
        self.filteredOdomSub = rospy.Subscriber('/odom/filtered', Odometry, self.filteredOdomCb)
        # Two levels up, if map localization is running (at low freq).
        self.mapPoseSub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.mapPoseCb)
        #self.worldPoseLinCov = 0.
        #self.worldPoseAngCov = 0.
        self.worldPose = Odometry()
        self.worldPose.child_frame_id = robotFrame
        self.worldPose.header.frame_id = worldFrame
        self.worldPosePub = rospy.Publisher(worldPoseTopic, Odometry, queue_size =1)
        self.transform = TransformStamped()
        self.filteredOdomMsg = Odometry()
        self.filteredOdomMsg.header.frame_id = odomFrame
        self.filteredOdomMsg.child_frame_id = robotFrame
        self.wheelOdomMsg = Odometry()
        self.wheelOdomMsg.header.frame_id = odomFrame
        self.wheelOdomMsg.child_frame_id = robotFrame
        self.mapPoseMsg = PoseWithCovarianceStamped()
        self.mapPoseMsg.header.frame_id = worldFrame
        
        self.newGoalEvent = threading.Event()
        self.currentActionGoal = LowLevelNavigationGoal()
        self.currentActionFeedback = LowLevelNavigationFeedback()
        self.goalDoneEvent = threading.Event()
        self.currentActionResult = LowLevelNavigationResult()

    def filteredOdomCb(self, msg):
        self.filteredOdomMsg = msg

    def mapPoseCb(self, msg):
        self.mapPoseMsg = msg
        #self.mapPoseLinCov = max(msg.pose.covariance[0], msg.pose.covariance[7])
        #self.mapPoseAngCov = msg.pose.covariance[35]

    def wheelOdomCb(self, msg):
        self.wheelOdomMsg = msg

    def updateWorldPose(self, publish=True):
        timeNow = rospy.Time.now()
        filteredOdomDt = (timeNow - self.filteredOdomMsg.header.stamp).to_sec()
        if filteredOdomDt <= self.filteredOdomTimeout:
            odomMsg = self.filteredOdomMsg
        else:
            # fallback to unfiltered odom
            odomMsg = self.wheelOdomMsg
            print("WARNING: Staleness of filtered odometry", filteredOdomDt, "s older than desired",
                        self.filteredOdomTimeout, "s. Using wheel odometry with higher covariance instead.")
        self.worldPose.header.stamp = odomMsg.header.stamp
        if self.tfBuffer.can_transform(self.worldPose.header.frame_id, 
                                       odomMsg.header.frame_id,
                                       rospy.Time(0), rospy.Duration(self.mapPoseTimeout)):
            # assembling world pose from filtered odom msg and map pose
            self.transform = self.tfBuffer.lookup_transform(self.worldPose.header.frame_id, 
                                                            odomMsg.header.frame_id,
                                                            rospy.Time(0))
        else:
            print("ERROR: Transform", odomMsg.header.frame_id, "->",
                    self.worldPose.header.frame_id, " is outdated. Drifting is not corrected.")
        pose = PoseStamped()
        pose.header = odomMsg.header
        pose.pose = odomMsg.pose.pose
        # transform odom pose to world pose.
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, self.transform)
        self.worldPose.pose.pose = pose_transformed.pose
        self.worldPose.pose.covariance = utils.composeHTMCov(odomMsg.pose.covariance,
                                                            self.transform,
                                                            self.mapPoseMsg.pose.covariance).tolist()
        # assemble velocity
        if self.worldPose.child_frame_id != odomMsg.child_frame_id:
            if self.tfBuffer.can_transform(self.worldPose.child_frame_id,
                                            odomMsg.child_frame_id, rospy.Time(0)):
                tfVel = self.tfBuffer.lookup_transform(self.worldPose.child_frame_id,
                                            odomMsg.child_frame_id, rospy.Time(0))
                self.worldPose.twist.twist = tf2_geometry_msgs.do_transform_vector3(odomMsg.twist.twist, tfVel)
                self.worldPose.twist.covariance = utils.composeRotCov(odomMsg.twist.covariance, tfVel).tolist()
            else:
                print("ERROR: Timed out interpolating Twist from", odomMsg.child_frame_id, "to",
                        self.worldPose.child_frame_id, ".")
        else:
            self.worldPose.twist = odomMsg.twist
        if publish:
            self.worldPosePub.publish(self.worldPose)

    def getWorldPose(self, timeTol=0.1, publish=True):
        timeNow = rospy.Time.now()
        if (timeNow - self.worldPose.header.stamp).to_sec() > timeTol:
            # Do update
            self.updateWorldPose(publish)
        return self.worldPose

    """A low-level goal runner. Tries to reach a goal with specified uncertainty tolerance"""
    def runGoal(self, actionGoal):
        # push goal
        self.currentActionGoal = actionGoal
        # clear previous records
        self.currectActionFeedback = None
        self.currentActionResult = None
        # set event
        self.newGoalEvent.set()
        # determine which module to turn on (and its dependency)
        # turn on/off the module
        # when the planner module is turned on, it submits an action goal to the movebase planner.
        # The planner guides the robot to reach the goal and return success.
        # if the goal fails, the decider switches to another module.
        # block until completes/fails
        self.goalDoneEvent.wait()
        self.goalDoneEvent.clear()
        return self.currentActionResult
