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
from ReFrESH_ROS_utils import Ftype, Launcher, ROSTopicMonitor, ROSnodeMonitor
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from omniveyor_common.msg import LowLevelPoseAction, LowLevelPoseGoal, LowLevelPoseResult, LowLevelPoseFeedback, LowLevelPoseActionGoal
from actionlib_msgs.msg import GoalStatus
from PlannerModules import PlannerModule
from utils import *
import actionlib

class MoveBaseManager(PlannerModule, Manager):
    """Multilevel inherited class to manage movebase global and local planners

    Args:
        PlannerModule (_type_): _description_
        Manager (_type_): _description_
    """
    def __init__(self, launcher:Launcher, managedModules:list=[], name:str="moveBaseManager", priority:int=97,
                preemptive:bool=True, freq:float=5.0, minReconfigInterval:float=1.0,
                mapTopic="map", mapFrame="map", refFrame="base_link"):
        """Manager class for MoveBase Planners as ReFrESH Modules

        Args:
            launcher (Launcher): ROS Launcher
            managedModules (list, optional): ReFrESH modules registered to this manager. Defaults to [].
            name (str, optional): Registered name of this manager to upper-level manager. Defaults to "moveBaseManager".
            priority (int, optional): Priority of starting the MoveBase compared to other motion planning methods. Defaults to 97.
            preemptive (bool, optional): Is the MoveBase and associated planners preempting other motion planners. Defaults to True.
            freq (float, optional): Frequency that monitors the MoveBase planners' performance. Defaults to 5.0.
            minReconfigInterval (float, optional): Minimum interval of switching between planners. Defaults to 1.0.
        """
        PlannerModule.__init__(self, name, priority, preemptive, EX_thread=4, EV_thread=1, ES_thread=1)
        Manager.__init__(self, launcher, managedModules, name, freq, minReconfigInterval)
        self.EX[0] = self.Decider
        self.setComponentProperties('EX', Ftype.LAUNCH_FILE, 'omniveyor_mobility', 'navigation.launch', ind=1)
        self.setComponentProperties('EX', Ftype.THREAD, exec=self.updateGoal, pre=self.moveRobotIntoCostmap, ind=2)
        self.setComponentProperties('EX', Ftype.THREAD, exec=self.setTerminalState, ind=3)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 5.0})
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)
        # Resource metric: CPU time, memory, topics
        self.resourceMetrics = [0.5, 0.0, 0.0]
        # Performance metric: metric bottleneck of submodules. This metric is cleared once the goal pose change.
        # The performance metric is a member of the manager class.
        self.cpuQuota = 0.5
        self.memQuota = 0.5
        self.exMon = ROSnodeMonitor()
        self.mapTopic = mapTopic
        self.mapFrame = mapFrame
        self.refFrame = refFrame
        self.topicMon = ROSTopicMonitor(subs=[[mapTopic, OccupancyGrid]], tf=[[mapFrame,refFrame]])
        self.robotInMap = False
        self.goalDoneEvent = threading.Event()
        self.isNewGoal = False
        # The class instance is mutable (call by reference)

    def translate(self, goalIn:LowLevelPoseGoal):
        """Translate slot type class to dict type class, such that custom field manipulation is possible

        Args:
            goalIn (LowLevelPoseGoal): Input slot type class, usually passed from ROS message

        Returns:
            _type_: Dict type class of the action goal, with MoveBase Goal appended to the property
        """
        unpackedGoal = super().translate(goalIn)
        if not hasattr(unpackedGoal, 'move_base_goal'):
            setattr(unpackedGoal, 'move_base_goal',
                MoveBaseGoal(target_pose=PoseStamped(header=goalIn.header, pose=goalIn.target_pose.pose)))
        # consistent with parent class
        return unpackedGoal

    def submit(self):
        """Submit self.currentActionGoal to the planner that is currently on, or the one with best performance
        """
        super().submit()
        # update goals of the modules.
        for m in self.moduleDict:
            m.updateGoal(compare=True, submit=False)
        # check if any of the managed module is on
        if not len(self.onDict):
            # update estimator values
            for m in self.offDict:
                m.estimator()
            # if none is on, turn on the one with best performance.
            self.turnOnBestESPerf()
        # submit the goal on the turned on module.
        for m in self.onDict:
            m.submit()

    def cancel(self):
        super().cancel()
        for m in self.onDict:
            m.cancel()

    def moveRobotIntoCostmap(self):
        # map = wait_for_msg(self.mapTopic)
        if self.robotInMap:
            return
        print('INFO: Rotating in-place to move robot into costmap...')
        initTime = rospy.Time.now()
        rotateCmdPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        slowRotateCmd = Twist()
        slowRotateCmd.linear.x = 0.; slowRotateCmd.linear.y = 0.; slowRotateCmd.angular.z = 0.1
        while not rospy.is_shutdown():
            map = rospy.wait_for_message(self.mapTopic, OccupancyGrid)
            # assume map is always upright
            mapVerts = [[map.info.origin.position.x, map.info.origin.position.y]]
            mapVerts.append([mapVerts[0][0]+map.info.height*map.info.resolution, mapVerts[0][1]])
            mapVerts.append([mapVerts[1][0], mapVerts[0][1]+map.info.width*map.info.resolution])
            mapVerts.append([mapVerts[0][0], mapVerts[0][1]+map.info.width*map.info.resolution])
            self.robotInMap = ptInRectangle(self.getPoseInGoalFrame().pose.pose.position, mapVerts)
            if self.robotInMap:
                slowRotateCmd.angular.z = 0.0
                rotateCmdPub.publish(slowRotateCmd)
                break
            if (rospy.Time.now()-initTime).to_sec() >=16:
                slowRotateCmd.angular.z = 0.0
                rotateCmdPub.publish(slowRotateCmd)
                self.resourceMetrics[0] = 1.0
                self.resourceMetrics[1] = 1.0
                self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
                break
            # publish [0., 0., 0.1] to cmd_vel
            rotateCmdPub.publish(slowRotateCmd)
        print('INFO: Robot is inside costmap.')

    def updateGoal(self, goal:LowLevelPoseGoal=None, compare:bool=True, submit:bool=True):
        """Update locally stored goal

        Args:
            goal (LowLevelPoseGoal, optional): input goal. Defaults to None.
            compare (bool, optional): Whether to compare the input goal with previous goal. Defaults to True.
            submit (bool, optional): Whether to submit the goal to managed planners. Defaults to True.
        """
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
        """Called when a task completes. If it succeeded, report goal succeess to upper-level manager.
        If all managed modules fail, append failed goal, pose at failure, and time at failure to local
        record, and report goal abortion to upper-level manager
        """
        while not rospy.is_shutdown():
            self.goalDoneEvent.wait(1.0)
            if not self.goalDoneEvent.isSet():
                continue
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
        """Periodic evaluation of the performance and resource consumption of currently active planner + MoveBase

        Args:
            event (_type_): Timer event, triggered periodically
        """
        if not self.topicMon.isAttached():
            self.topicMon.attach(self)
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
        """On-demand estimation of how MoveBase can potentially perform. Called without MoveBase running.
        """
        if not self.topicMon.isAttached():
            self.topicMon.attach(self)
        if not all(self.topicMon.subsHaveSources()):
            # the topic setup is insufficient for navigation in the map.
            self.resourceMetrics[2] = 1.0
            # we are done here.
            self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
            return
        if not all(self.topicMon.tfFeasible()):
            # the tf setup is insufficient for navigation in the map.
            self.resourceMetrics[2] = 1.0
            # we are done here.
            self.reconfigMetric.update([self.bottleNeck], self.resourceMetrics)
            return
        self.resourceMetrics[2] = 0.0
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

    def findBestESPerf(self, exclude:tuple=()):
        """Modified function to find the most performant planner module. Cancels all active goals since
        the process requires to reconfig MoveBase parameters for each planner setup. Called when MoveBase
        is active but NO Planner Module is active 

        Args:
            exclude (tuple, optional): Modules to exclude. Defaults to ().

        Returns:
            _type_: Tuple containing the module and estimated performance
        """
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
                RuntimeWarning(str(item)+"goal cancelled to run ES of other modules.")
        return super().findBestESPerf(exclude)

class MotionManager(Manager):
    def __init__(self, launcher:Launcher, managedModules:list=[], name:str="MotionManager", freq:float=5.0, minReconfigInterval:float=1.0,
                 mapRef:bool=True, odomRef:bool=True, objectRef:bool=True, timeout:float=0.1, refFrame:str="base_link"):
        super().__init__(launcher, managedModules, name, freq, minReconfigInterval)
        # handle was previously installed to super(). Need to migrate to self for the additional functions.
        #for m in managedModules:
        #    m.managerHandle = self
        # utilities that interpolates robot's pose in the world (map frame, with fallback to odom frame)
        # tf - transform odom to map frame for amcl_pose, filtered odom,
        # fallback for other map poses published through tf.
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
        self.currentActionGoal = LowLevelPoseGoal()
        self.goalFeedback = PoseStamped()
        self.goalDoneEvent = threading.Event()
        self.goalStatus = GoalStatus.RECALLED

        self.runGoalActionSetup()
        self.goal_id_tracker = 0

    def runGoalActionSetup(self):
        self.runActionSrv = actionlib.SimpleActionServer(self.name+"/run_goal_action",
            LowLevelPoseAction, execute_cb=self._runGoal, auto_start = False)
        self.runActionSrv.start()

    def odomPoseRawCb(self, msg:Odometry):
        self.rawOdomPose = msg

    def odomPoseFilteredCb(self, msg:Odometry):
        self.filteredOdomPose = msg

    def mapPoseRawCb(self, msg:PoseWithCovarianceStamped):
        self.rawMapPose = msg

    def mapPoseFilteredCb(self, msg:Odometry):
        self.filteredMapPose = msg

    def objectPoseCb(self, msg:Odometry):
        self.objectPose = msg
    
    def getPoseInGoalFrame(self):
        """Gets robot pose in the the frame same as currentActionGoal.header.frame_id. 
        Either the header.frame_id==goal.header.frame_id, or child_frame_id==goal.header.frame_id

        Returns:
            Odometry | PoseWithCovarianceStamped | None: Relevant Pose
        """
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

    def getRelativePoseStamped(self)->tuple:
        # get measurement source
        # get tf from source frame to base frame
        # transform pose to base frame
        pose_ = self.getPoseInGoalFrame()
        if not pose_:
            return None, None, None
        # pose_ is already relevant with goal. Check if it's the header or child frame. If the pose msg
        # does not have child_frame_id, assume it is the refFrame, a frame with the robot.
        ps_ = PoseStamped(header=pose_.header, pose=pose_.pose.pose)
        tf_ = TransformStamped(header=pose_.header, child_frame_id=self.refFrame)
        tf_.transform.rotation.w = 1.
        # the observed pose is already from base to object, relative to the base. return as-is.
        if pose_.header.frame_id == self.refFrame:
            return ps_, tf_, pose_
        # the observed pose is measured in the goal.header frame. transform goal to base frame.
        psg_ = PoseStamped(pose=self.currentActionGoal.target_pose.pose)
        psg_.header.frame_id = self.currentActionGoal.header.frame_id
        tf_, isUpdated = updateTransform(tf_, self.tfBuffer, self.staleTol)
        if isUpdated:
            # in case the tf is stale, transform the pose over as well.
            psg_ = tf2_geometry_msgs.do_transform_pose(psg_, tf_)
            if ps_.header.stamp > tf_.header.stamp:
                # a newer msg than tf. Is this really necessary?
                ps_ = tf2_geometry_msgs.do_transform_pose(ps_, tf_)
                psg_.pose.position.x -= ps_.pose.position.x
                psg_.pose.position.y -= ps_.pose.position.y
                psg_.pose.position.z -= ps_.pose.position.z
                psg_.pose.orientation = quaternionDiff(psg_.pose.orientation, ps_.pose.orientation)
            return psg_, tf_, pose_
        else:
            return None, None, pose_

    def getRelativePoseWithCovarianceStamped(self)->PoseWithCovarianceStamped:
        # getRelativePose
        ps_, tf_, pose_ = self.getRelativePoseStamped()
        # transform covariance. Assume tf doesn't involve uncertainty.
        pcs = PoseWithCovarianceStamped(header=ps_.header)
        pcs.pose.pose = ps_.pose
        pcs.pose.covariance = composeHTMCov(pose_.pose.covariance, tf_)
        return pcs

    def getRemainingDistance(self, poseInGoalFrame:PoseWithCovarianceStamped=None):
        if not poseInGoalFrame:
            relPose = self.getRelativePoseStamped()
            if relPose[0] is None:
                return -1., -1.
            relPose = relPose[0].pose
            #print(relPose)
        else:
            relPose = poseInGoalFrame
        if not relPose:
            return -1., -1.
        linDist = np.linalg.norm([relPose.position.x, relPose.position.y, relPose.position.z])
        angDist = abs(rpyFromQuaternion(relPose.orientation)[2])
        #print(linDist, angDist)
        return linDist, angDist

    def getRemainingTime(self):
        return (self.currentActionGoal.expiration - rospy.Time.now()).to_sec()

    """A low-level goal runner. Tries to reach a goal with specified uncertainty tolerance"""
    def runGoal(self, pose:PoseStamped=PoseStamped(), tolerance:list=[0.1, 0.1],
                    timeout:float=30.0, actionGoal:LowLevelPoseGoal=None):
        if actionGoal is not None:
            # push goal
            currentActionGoal = actionGoal
        else:
            currentActionGoal = LowLevelPoseGoal()
            currentActionGoal.header = pose.header
            if not currentActionGoal.header.frame_id:
                currentActionGoal.header.frame_id = 'map'
            currentActionGoal.target_pose.pose = pose.pose
            cov = toleranceToCov([tolerance[0], tolerance[0], tolerance[0],
                                tolerance[1], tolerance[1], tolerance[1]])
            currentActionGoal.target_pose.covariance = cov.tolist()
            currentActionGoal.expiration = rospy.Time.now()+rospy.Duration(timeout)
        # execution path is not through action server. Notify action server.
        tmpActionGoal = LowLevelPoseActionGoal(header=currentActionGoal.header, goal=currentActionGoal)
        tmpActionGoal.goal_id.stamp = currentActionGoal.header.stamp
        tmpActionGoal.goal_id.id = "%s-%i-%.3f" % (self.name, self.goal_id_tracker, currentActionGoal.header.stamp.to_sec())
        self.goal_id_tracker += 1
        self.runActionSrv.action_server.internal_goal_callback(tmpActionGoal)
        self.goalStatus = GoalStatus.ACTIVE
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            #print(self.goalStatus)
            if self.goalStatus in \
                [GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                break
        return self.goalStatus

    """Wrapped version inside the goal server"""
    def _runGoal(self, actionGoal):
        self.currentActionGoal = actionGoal
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
        # block until completes/fails]
        while not rospy.is_shutdown():
            if self.runActionSrv.is_preempt_requested():
                self.runActionSrv.set_preempted(LowLevelPoseResult(self.goalFeedback))
                for m in self.onDict:
                    if not hasattr(m, 'cancel'):
                        continue
                    if not callable(m.cancel):
                        continue
                    m.cancel()
                self.goalStatus = GoalStatus.PREEMPTED
                break
            self.goalDoneEvent.wait(0.1)
            self.runActionSrv.publish_feedback(LowLevelPoseFeedback(self.goalFeedback))
            if self.goalDoneEvent.isSet():
                if self.goalStatus == GoalStatus.SUCCEEDED:
                    self.runActionSrv.set_succeeded(LowLevelPoseResult(self.goalFeedback))
                elif self.goalStatus in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
                    self.runActionSrv.set_preempted(LowLevelPoseResult(self.goalFeedback))
                else:
                    self.runActionSrv.set_aborted(LowLevelPoseResult(self.goalFeedback))
                break
        self.goalDoneEvent.clear()

    def shutdown(self):
        super().shutdown()
        self.goalDoneEvent.set()
