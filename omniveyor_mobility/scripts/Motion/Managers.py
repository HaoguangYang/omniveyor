#!/usr/bin/python

import rospy
import os
import sys
import numpy as np
import threading
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from ReFrESH_ROS import Manager
from ReFrESH_ROS_utils import Ftype, ROSnodeMonitor
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal
from omniveyor_common.msg import LowLevelPoseActionGoal, LowLevelPoseActionFeedback, LowLevelPoseActionResult
from actionlib_msgs.msg import GoalStatus
from PlannerModules import PlannerModule
from utils import *

"""multilevel inherited class to manage movebase global and local planners"""
class MoveBaseManager(PlannerModule, Manager):
    def __init__(self, launcher, managedModules=[], name="moveBaseManager", priority=97, preemptive=True,
                freq=5.0, minReconfigInterval = 1.0):
        PlannerModule.__init__(self, name, priority, preemptive, EX_thread=4, EV_thread=1, ES_thread=1)
        Manager.__init__(self, launcher, managedModules, name, freq, minReconfigInterval)
        self.EX[0] = self.Decider
        self.setComponentProperties('EX', Ftype.LAUNCH_FILE, 'omniveyor_mobility', 'navigation.launch', ind=1)
        self.setComponentProperties('EX', Ftype.THREAD, exec=self.updateGoal, ind=2)
        self.setComponentProperties('EX', Ftype.THREAD, exec=self.setTerminalState, ind=3)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 5.0})
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)
        # Resource metric: CPU time, memory
        self.resourceMetrics = [0.5, 0.0]
        # Performance metric: metric bottleneck of submodules. This metric is cleared once the goal pose change.
        # The performance metric is a member of the manager class.
        self.cpuQuota = 0.5
        self.memQuota = 0.5
        self.exMon = ROSnodeMonitor()
        self.goalDoneEvent = threading.Event()
        self.isNewGoal = False
        # The class instance is mutable (call by reference)

    def translate(self, goalIn):
        unpackedGoal = super().translate(goalIn)
        if not hasattr(unpackedGoal, 'move_base_goal'):
            setattr(unpackedGoal, 'move_base_goal',
                    MoveBaseGoal(target_pose=PoseStamped(header=goalIn.header, pose=goalIn.target_pose.pose)))
        # consistent with parent class
        return unpackedGoal

    def submit(self):
        super().submit()
        # check if any of the managed module is on
        if not len(self.onDict):
            # update goals of the modules.
            for m in self.moduleDict:
                m.updateGoal(compare=True, submit=False)
                m.estimator()
            # if not, turn on the one with best performance.
            self.turnOnBestESPerf()
        # submit the goal on the turned on module.
        for m in self.onDict:
            m.submit()

    def updateGoal(self, goal=None, compare=True, submit=True):
        try:
            while not rospy.is_shutdown():
                # blocking call: wait until a new goal is available, and update local goal.
                # call submit function after updating.
                isNewGoal = super().updateGoal(goal, compare, submit)
                if compare:
                    self.isNewGoal = isNewGoal
                if not self.managerHandle.moduleIsOn(self):
                    break
        except SystemExit:
            self.turnOffAll()

    def setTerminalState(self):
        while not rospy.is_shutdown():
            self.goalDoneEvent.wait()
            self.goalDoneEvent.clear()
            if self.goalStatus == GoalStatus.SUCCEEDED:
                self.reportTerminalState()
                continue
            elif self.goalStatus == GoalStatus.PREEMPTED:
                # in the middle of updateGoal. another (updated) goal awaits.
                continue
            elif self.bottleNeck > 1.:
                # none of the registered modules can complete the task.
                record = AbortionRecord(self.currentActionGoal, self.getPoseInGoalFrame())
                self.abortedGoals.append(record)
                self.reportTerminalState()
            if not self.managerHandle.moduleIsOn(self):
                break

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
        self.updateGoal(compare=True, submit=False)
        # movebase is off. obtain a-priori esitmates
        if self.bottleNeck >= 1.0:
            # we had a failed record before
            # the base can rotate in place, so goal pose is less important in determining success.
            if self.isNewGoal:
                # we are receiving a goal at a new location. Reset the estimators
                self.bottleNeck = 0.0
                self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
        else:
            # this module succeeded last time. Keep using it.
            self.bottleNeck = 0.0
            self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)

    def findBestESPerf(self, exclude=()):
        for _, (exHandle, _) in self.onDict.items():
            for item in exHandle:
                # not the right handle
                if not hasattr(item, 'cancel_all_goals'):
                    continue
                if not callable(item.cancel_all_goals):
                    continue
                # cancel all goals previously submitted, since calling module ES requires
                # reconfiguring MoveBase parameters.
                item.cancel_all_goals()
        return super().findBestESPerf(exclude)

class MotionManager(Manager):
    def __init__(self, launcher, managedModules=[], name="MotionManager", freq=5.0, minReconfigInterval = 1.0,
                 mapRef = True, odomRef = True, objectRef = True, timeout=0.1, refFrame="base_link"):
        super().__init__(launcher, managedModules, name, freq, minReconfigInterval)
        # handle was previously installed to super(). Need to migrate to self for the additional functions.
        #for m in managedModules:
        #    m.managerHandle = self
        # utilities that interpolates robot's pose in the world (map frame, with fallback to odom frame)
        # tf - transform odom to map frame for amcl_pose, filtered odom,
        # fallback for other map poses published through tf.
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.staleTol = timeout
        
        if odomRef:
            self.odomPoseSub = rospy.Subscriber('odom', Odometry, self.odomPoseRawCb)
            self.odomPoseFSub = rospy.Subscriber('odom/filtered', Odometry, self.odomPoseFilteredCb)
        if mapRef:
            self.mapPoseSub = rospy.Subscriber('map_pose', PoseWithCovarianceStamped, self.mapPoseRawCb)
            self.mapPoseFSub = rospy.Subscriber('map_pose/filtered', Odometry, self.mapPoseFilteredCb)
        if objectRef:
            self.objectPoseSub = rospy.Subscriber('object_detection/pose', Odometry, self.objectPoseCb)
        
        self.refFrame = refFrame
        self.rawOdomPose = Odometry()
        self.filteredOdomPose = Odometry()
        self.rawMapPose = PoseWithCovarianceStamped()
        self.filteredMapPose = Odometry()
        # maybe twist is not so important, but we need child_frame.
        self.objectPose = Odometry()
        
        self.newGoalEvent = threading.Event()
        self.currentActionGoal = LowLevelPoseActionGoal()
        self.currentActionFeedback = LowLevelPoseActionFeedback()
        self.goalDoneEvent = threading.Event()
        self.currentActionResult = LowLevelPoseActionResult()

    def odomPoseRawCb(self, msg):
        self.rawOdomPose = msg

    def odomPoseFilteredCb(self, msg):
        self.filteredOdomPose = msg

    def mapPoseRawCb(self, msg):
        self.rawMapPose = msg

    def mapPoseFilteredCb(self, msg):
        self.filteredMapPose = msg

    def objectPoseCb(self, msg):
        self.objectPose = msg
    
    def getPoseInGoalFrame(self):
        # check the frame that the goal is expressed in
        # check the timeout tolerance for the corresponding variables
        # return the apropriate variable.
        timeNow = rospy.Time.now()
        if self.currentActionGoal.header.frame_id == self.filteredMapPose.header.frame_id:
            if (timeNow - self.filteredMapPose.header.stamp).to_sec() < self.staleTol:
                return self.filteredMapPose
            elif self.currentActionGoal.header.frame_id == self.rawMapPose.header.frame_id:
                return self.rawMapPose
            else:
                RuntimeWarning("Filtered map pose is too stale, but raw map pose has different frames!")
                return self.filteredMapPose
        elif self.currentActionGoal.header.frame_id == self.filteredOdomPose.header.frame_id:
            if (timeNow - self.filteredOdomPose.header.stamp).to_sec() < self.staleTol:
                return self.filteredOdomPose
            elif self.currentActionGoal.header.frame_id == self.rawOdomPose.header.frame_id:
                return self.rawOdomPose
            else:
                RuntimeWarning("Filtered odom pose is too stale, but raw odom pose has different frames!")
                return self.filteredOdomPose
        # it's the high level planner's duty to hand over the goal to object frame only when the object is detectable.
        elif self.currentActionGoal.header.frame_id in \
                [self.objectPose.header.frame_id, self.objectPose.child_frame_id]:
            return self.objectPose
        # fallbacks
        elif self.currentActionGoal.header.frame_id == self.rawMapPose.header.frame_id:
            return self.rawMapPose
        elif self.currentActionGoal.header.frame_id == self.rawOdomPose.header.frame_id:
            return self.rawOdomPose
        else:
            RuntimeError("Current Action Goal is in non of the recognized map/odom/object frames!")
            return None

    def getRelativePoseStamped(self):
        # get measurement source
        # get tf from source frame to base frame
        # transform pose to base frame
        pose_ = self.getPoseInGoalFrame()
        if not pose_:
            return None, None, None
        ps_ = PoseStamped(header=pose_.header, pose=pose_.pose.pose)
        tf_ = TransformStamped(header=pose_.header, child_frame_id=self.refFrame)
        tf_.transform.rotation.w = 1.
        if pose_.header.frame_id == self.refFrame:
            return ps_, tf_, pose_
        tf_, isUpdated = updateTransform(tf_, self.tfBuffer, self.staleTol)
        if isUpdated:
            return tf2_geometry_msgs.do_transform_pose(ps_, tf_), tf_, pose_
        else:
            return None, None, pose_

    def getRelativePoseWithCovarianceStamped(self):
        # getRelativePose
        ps_, tf_, pose_ = self.getRelativePoseStamped()
        # transform covariance. Assume tf doesn't involve uncertainty.
        pcs = PoseWithCovarianceStamped(header=ps_.header)
        pcs.pose.pose = ps_.pose
        pcs.pose.covariance = composeHTMCov(pose_.pose.covariance, tf_)
        return pcs

    def getRemainingDistance(self, poseInGoalFrame=None):
        if not poseInGoalFrame:
            relPose = self.getRelativePoseStamped()[0].pose
        else:
            relPose = poseInGoalFrame
        if not relPose:
            return -1., -1.
        linDist = np.linalg.norm(linearDiff(self.currentActionGoal.target_pose.pose.position, relPose.position))
        angDist = abs(angleDiff(rpyFromQuaternion(relPose.orientation)[2], 
                                rpyFromQuaternion(self.currentActionGoal.target_pose.pose.orientation)[2]))
        return linDist, angDist

    def getRemainingTime(self):
        return (self.currentActionGoal.expiration - rospy.Time.now()).to_sec()

    """A low-level goal runner. Tries to reach a goal with specified uncertainty tolerance"""
    def runGoal(self, pose:PoseStamped=PoseStamped(), 
                    tolerance=[0.1, 0.1], timeout=30.0, actionGoal=None):
        if actionGoal is not None:
            # push goal
            self.currentActionGoal = actionGoal
        else:
            self.currentActionGoal = LowLevelPoseActionGoal()
            self.currentActionGoal.header = pose.header
            if not self.currentActionGoal.header.frame_id:
                self.currentActionGoal.header.frame_id = 'map'
            self.currentActionGoal.target_pose.pose = pose.pose
            cov = toleranceToCov([tolerance[0], tolerance[0], tolerance[0],
                                tolerance[1], tolerance[1], tolerance[1]])
            self.currentActionGoal.target_pose.covariance = cov.tolist()
            self.currentActionGoal.expiration = rospy.Time.now()+rospy.Duration(timeout)
        # clear previous records
        self.currectActionFeedback = None
        self.currentActionResult = None
        # set event
        self.newGoalEvent.set()
        # turn on the one with best estimated performance, if no subordinate module is running.
        if not len(self.onDict):
            self.turnOnBestESPerf()
        # determine which module to turn on (and its dependency)
        # turn on/off the module
        # when the planner module is turned on, it submits an action goal to the movebase planner.
        # The planner guides the robot to reach the goal and return success.
        # if the goal fails, the decider switches to another module.
        # block until completes/fails
        self.goalDoneEvent.wait(timeout+1.0)
        self.goalDoneEvent.clear()
        return self.currentActionResult

    def shutdown(self):
        super().shutdown()
        self.goalDoneEvent.set()
