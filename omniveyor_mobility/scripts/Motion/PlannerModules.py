#!/usr/bin/python

import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import copy
import threading
import rospy
import dynamic_reconfigure.client
from dynamic_reconfigure import DynamicReconfigureCallbackException
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from ReFrESH_ROS import ReFrESH_Module
from ReFrESH_ROS_utils import Ftype, RingBuffer, DictObj
from utils import *

"""
Prototype class that manipulates planners such as MoveBase. Provides additional functions for
self adaptation, e.g. for unpacking goal expression, for determining if two goals are the same.
"""
class PlannerModule(ReFrESH_Module):
    def __init__(self, name="plannedMotion", priority=90, preemptive=True, retry=2, 
                    goalHistoryLength=1, EX_thread=1, EV_thread=1, ES_thread=1):
        super().__init__(name, priority, preemptive, 
                        EX_thread=EX_thread, EV_thread=EV_thread, ES_thread=ES_thread)
        self.currentActionGoal = None
        self.goalStatus = GoalStatus.RECALLED
        # a buffer that tracks where the planner has failed.
        self.abortedGoals = RingBuffer(goalHistoryLength)
        self.feedbackPose = None
        self.max_retry=retry
        self.retries=0

    """Prototype function to submit the goal. Starts goal execution. No blocking."""
    def submit(self):
        pass

    """Prototype function to cancel the goal."""
    def cancel(self):
        self.goalStatus = GoalStatus.PREEMPTED

    """Prototype function to retry execution of the current goal."""
    def retry(self):
        print("WARNING: Goal failed to reach. Retrying...")
        self.submit()

    """This function sets the current pose of the robot. 
    It is critical if the module is run out of context i.e. the manager does not have
    the getPoseInGoalFrame() funtion."""
    def setPoseFeedback(self, fb):
        self.feedbackPose = fb
        if hasattr(self.managerHandle, 'goalFeedback'):
            self.managerHandle.goalFeedback = self.feedbackPose

    """ Fetch a goal from the manager, or from a given data structure. Store locally.
    Returns if the goal is different from previous goal."""
    def updateGoal(self, goal=None, compare=True, submit=False):
        print("INFO: Updating Local Goal in Module", self.name, ".")
        if goal is not None:
            newActionGoal = self.translate(goal)
        elif hasattr(self.managerHandle, 'currentActionGoal'):
            if hasattr(self.managerHandle, 'newGoalEvent'):
                if isinstance(self.managerHandle.newGoalEvent, threading.Event):
                    # stuck here to wait for new goal to arrive
                    while not rospy.is_shutdown():
                        self.managerHandle.newGoalEvent.wait(1.0)
                        if self.managerHandle.newGoalEvent.isSet():
                            break
                    self.managerHandle.newGoalEvent.clear()
            newActionGoal = self.translate(self.managerHandle.currentActionGoal)
        else:
            raise RuntimeError('Provided goal to'+self.name+'is invalid.')
        # is the goal different?
        if not compare:
            return True
        isDifferent = not self.compare(newActionGoal, self.currentActionGoal)
        self.currentActionGoal = newActionGoal
        self.goalStatus = GoalStatus.PENDING
        if submit:
            self.submit()
        return isDifferent
        # if the goal is different, up to the inherited class to do further operations.

    """Unpack goal tolerance from the data structure to prevent repeated calculation."""
    def translate(self, goalIn):
        # A prototype function. extract tolerance information from covariance.
        if goalIn is None:
            return None
        if hasattr(goalIn, '__slots__'):
            # a raw ROS message.
            tmpDict = {}
            # we only unpack one level, since the underlying structure is still
            # useful for reconstruction.
            for field in goalIn.__slots__:
                tmpDict.update({field: getattr(goalIn, field)})
            unpackedGoal = DictObj(tmpDict)
        else:
            unpackedGoal = copy.deepcopy(goalIn)
        if not hasattr(unpackedGoal, 'goal_tolerance') and hasattr(goalIn, 'target_pose'):
            if hasattr(goalIn.target_pose, 'covariance'):
                setattr(unpackedGoal, 'goal_tolerance', covToTolerance(goalIn.target_pose.covariance))
            else:
                setattr(unpackedGoal, 'goal_tolerance',(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]), np.eye(6)))
        return unpackedGoal

    """Get pose of robot in the map. Use manager method if available."""
    def getPoseInGoalFrame(self):
        if hasattr(self.managerHandle, 'getPoseInGoalFrame'):
            if callable(self.managerHandle.getPoseInGoalFrame):
                return self.managerHandle.getPoseInGoalFrame()
        return self.feedbackPose

    """Get remaining time of the current task before expiration.
    Use manager method if available."""
    def getRemainingTime(self):
        if hasattr(self.managerHandle, 'getRemainingTime'):
            if callable(self.managerHandle.getRemainingTime):
                return self.managerHandle.getRemainingTime()
        return (self.currentActionGoal.expiration - rospy.Time.now()).to_sec()

    """Get remaining distance from the target pose.
    Use manager method if available (that supports goal in different frames).
    Otherwise assume the goal is expressed in map frame."""
    def getRemainingDistance(self, poseInGoalFrame=None):
        if hasattr(self.managerHandle, 'getRemainingDistance'):
            if callable(self.managerHandle.getRemainingDistance):
                return self.managerHandle.getRemainingDistance(poseInGoalFrame)
        # assume feedbackPose is in the same frame with goal.
        if poseInGoalFrame is None:
            relPose = self.getPoseInGoalFrame().pose
        else:
            relPose = poseInGoalFrame
        #print(relPose)
        linDist = np.linalg.norm(linearDiff(
                self.currentActionGoal.target_pose.pose.position, relPose.position))
        angDist = abs(angleDiff(rpyFromQuaternion(relPose.orientation)[2], 
                rpyFromQuaternion(self.currentActionGoal.target_pose.pose.orientation)[2]))
        return linDist, angDist

    """Compare if two goals are the same. For self adaptation, a planner should NOT
    deal with a previously-failed goal until being explicitly reset."""
    def compare(self, newGoal, oldGoal, compareRef=True, comparePos=True, compareRot=False):
        # Return if the two goals are the same.
        if oldGoal is None or newGoal is None:
            return False
        if compareRef:
            if not self.compareReference(newGoal, oldGoal):
                return False
        if comparePos:
            if not self.comparePosition(newGoal.target_pose.pose, oldGoal.target_pose.pose,
                    tol=oldGoal.goal_tolerance[0][0]):
                return False
        if compareRot:
            if not self.compareOrientation(newGoal.target_pose.pose, oldGoal.target_pose.pose,
                    tol=oldGoal.goal_tolerance[0][3]):
                return False
        return True

    """Sub functions for goal comparison."""
    def compareReference(self, newGoal, oldGoal):
        return newGoal.header.frame_id == oldGoal.header.frame_id

    def comparePosition(self, newPose, oldPose, tol=0.1):
        linDist = np.linalg.norm(linearDiff(newPose.position, oldPose.position))
        if linDist > tol:
            return False
        return True

    def compareOrientation(self, newPose, oldPose, tol=0.1):
        rpyNew = rpyFromQuaternion(newPose.orientation)
        rpyOld = rpyFromQuaternion(oldPose.orientation)
        angDist = np.linalg.norm(angleDiff(rpyNew, rpyOld))
        if angDist > tol[1]:
            return False
        return True

    """When a goal fails, it keeps a local record (goal pose + current pose)."""
    def setTerminalState(self, status=GoalStatus.ABORTED, result=None):
        if status == GoalStatus.SUCCEEDED:
            dist = self.getRemainingDistance()
            #print(dist)
            if dist[0]>0 and dist[0]<self.currentActionGoal.goal_tolerance[0][0] and \
                dist[1]>0 and dist[1]<self.currentActionGoal.goal_tolerance[0][3]:
                # this goal is achieved
                self.goalStatus = GoalStatus.SUCCEEDED
                self.retries = 0
                self.reportTerminalState()
                return
            # this is a repeated SUCCEED message from last goal. Re-submit the goal.
            self.submit()
        if status == GoalStatus.PREEMPTED and self.goalStatus in [GoalStatus.PENDING, GoalStatus.PREEMPTED]:
            # in the middle of updateGoal, or upper level canceled the goal actively. another (updated) goal awaits.
            self.goalStatus = GoalStatus.PREEMPTED
            self.retries = 0
            self.reportTerminalState()
            return
        if self.retries < self.max_retry:
            self.retries += 1
            self.retry()
            return
        # this goal is canceled due to timeout and max retry has exceeded.
        record = AbortionRecord(self.currentActionGoal, self.getPoseInGoalFrame())
        self.abortedGoals.append(record)
        if status == GoalStatus.PREEMPTED:
            # escalate this.
            self.goalStatus = GoalStatus.ABORTED
        else:
            self.goalStatus = status
        self.reportTerminalState()

    """Report state of this goal as done. Set event handles on the manager"""
    def reportTerminalState(self):
        if hasattr(self.managerHandle, 'goalDoneEvent'):
            if isinstance(self.managerHandle.goalDoneEvent, threading.Event):
                self.managerHandle.goalDoneEvent.set()
        if hasattr(self.managerHandle, 'goalStatus'):
            self.managerHandle.goalStatus = self.goalStatus

    """function that determines if the current goal or current robot position has failed before."""
    def poseHasFailedBefore(self, compareGoalPose=True, compareCurrentPose=True):
        constTol = np.sqrt(np.array([0.1, 0.1])) * 3.
        # get current pose of robot
        curPose = self.getPoseInGoalFrame()
        # loop through records
        for record in self.abortedGoals.get():
            if compareGoalPose:
                if self.compare(self.currentActionGoal, record.goal):
                    # the goal pose is the same
                    return True
            if compareCurrentPose:
                if hasattr(record.abortedPose.pose, 'covariance'):
                    tol = covToTolerance(record.abortedPose.pose.covariance)
                    tol = np.array([tol[0], tol[3]]) * 3.
                else:
                    tol=constTol
                if self.comparePosition(record.abortedPose.pose, curPose.pose, tol=tol[0]):
                    # the robot pose is the same
                    return True
        return False

""" Manipulate Move Base planners """
class MoveBaseModule(PlannerModule):
    def __init__(self, name="moveBaseMotion", priority=90, preemptive=True, bgp="", blp="", kwargs:dict={}):
        super().__init__(name, priority, preemptive, EX_thread=1, EV_thread=1, ES_thread=1)
        # Tortuosity of path, time used, final error
        self.performanceMetrics = [0.0, 0.0]
        self.defaultPerformanceMetrics = [0.0, 0.0]
        self.tortuosityTol = 1.5
        self.speedLimit = [0.3, 0.3]
        # Resource metric: not-availability
        self.resourceMetrics = [0.5]
        self.setComponentProperties('EX', Ftype.ACTION_CLI, 'move_base', self.feedbackCb, mType=MoveBaseAction,
                            kwargs={'active_cb': self.activeCb, 'done_cb': self.setTerminalState,
                                    'prelaunch_cb': self.prelaunch, 'availTimeout': 1.0}, post=self.cancel)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0})
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)
        self.historyPoses = RingBuffer(50)
        # base global planner
        self.bgp = bgp
        # base local planner
        self.blp = blp
        self.otherCfg = kwargs
        self.isNewGoal = False

    def submit(self):
        print("INFO: Submitting Local Goal in Module", self.name, ".")
        super().submit()
        if not self.managerHandle.moduleIsOn(self):
            # module is off. do nothing.
            return
        # already an active module. Reusing it.
        exHandle = self.managerHandle.getEXhandle(self)
        # cancel ongoing goal and submit new goal
        for item in exHandle:
            # not the right handle
            if not hasattr(item, 'cancel_all_goals'):
                continue
            if not callable(item.cancel_all_goals):
                continue
            if not hasattr(item, 'submit'):
                continue
            if not callable(item.submit):
                continue
            # the action client handle is found
            # the last goal is canceled, invoking doneCb.
            # If another task is waiting, doneCb shall not push abortion record.
            item.cancel_all_goals()
            item.submit(self.currentActionGoal.move_base_goal)
            break

    def cancel(self):
        super().cancel()
        exHandle = self.managerHandle.getEXhandle(self)
        # cancel ongoing goal
        if exHandle is None:
            return
        for item in exHandle:
            # not the right handle
            if not hasattr(item, 'cancel_all_goals'):
                continue
            if not callable(item.cancel_all_goals):
                continue
            if not hasattr(item, 'submit'):
                continue
            if not callable(item.submit):
                continue
            # the action client handle is found
            # the last goal is canceled, invoking doneCb.
            # If another task is waiting, doneCb shall not push abortion record.
            item.cancel_all_goals()
            break

    def translate(self, goalIn):
        unpackedGoal = super().translate(goalIn)
        if not hasattr(unpackedGoal, 'move_base_goal'):
            setattr(unpackedGoal, 'move_base_goal',
                    MoveBaseGoal(target_pose=PoseStamped(header=goalIn.header, pose=goalIn.target_pose.pose)))
        # consistent with parent class
        return unpackedGoal

    def updateGoal(self, goal=None, compare=True, submit=False):
        isNewGoal = super().updateGoal(goal, compare, submit)
        if compare:
            self.isNewGoal = isNewGoal
        return isNewGoal

    def setInfeasible(self, which, ind):
        which[ind] = 1.0
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

    def prelaunch(self):
        # use dynamic reconfiguration to change the planner type to desired ones.
        #rospy.sleep(0.1)
        try:
            drc = dynamic_reconfigure.client.Client("move_base", timeout=10.0, config_callback=None)
        except rospy.ROSException:
            self.setInfeasible(self.resourceMetrics, 0)
            return
        #cfg = drc.get_configuration(timeout=2.0)
        #if not cfg:
        #    self.setInfeasible(self.resourceMetrics, 0)
        #    drc.close()
        #    return
        print("INFO: Updating MoveBase Global Planner to:", self.bgp, ", Local Planner to:", self.blp, ".")
        cfg = {'base_global_planner': self.bgp, 'base_local_planner': self.blp}
        cfg.update(self.otherCfg)
        try:
            drc.update_configuration(cfg)
        except DynamicReconfigureCallbackException:
            self.setInfeasible(self.resourceMetrics, 0)
            drc.close()
            return
        self.resourceMetrics[0] = 0.0
        drc.close()
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

    def feedbackCb(self, feedback):
        # synchronize feedback to this class
        pose = feedback.base_position
        self.setPoseFeedback(pose)
        # use raw quaternion since it's continuous.
        self.historyPoses.append([pose.header.stamp.to_sec(), pose.pose.position.x, pose.pose.position.y, 
                            pose.pose.orientation.z, pose.pose.orientation.w])

    def activeCb(self):
        self.goalStatus = GoalStatus.ACTIVE

    def evaluator(self, event):
        # /move_base/make_plan service available, distance to goal, time elapsed, turtuosity of path
        if self.currentActionGoal is None:
            return
        rowVect = self.historyPoses.get()
        self.performanceMetrics[0] = tortuosity(rowVect)/self.tortuosityTol
        remainingTime = self.getRemainingTime()
        if remainingTime:
            # still a valid task.
            e = self.getRemainingDistance()
            linE = max(e[0]-self.currentActionGoal.goal_tolerance[0][0], 0.)
            angE = max(e[1]-self.currentActionGoal.goal_tolerance[0][3], 0.)
            self.performanceMetrics[1] = max(linE/(self.speedLimit[0]*0.8), angE/(self.speedLimit[1]*0.8))/remainingTime
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
        else:
            # the task has expired. Set final status.
            self.setInfeasible(self.performanceMetrics, 1)
            self.turnMeOff()
            RuntimeWarning("Module "+self.name+" is Turning OFF. Reason: TIMEOUT")
            # cancel the task.
        # time_elapsed resets if distance_to_goal decreases every, say, 0.5m?

    def estimator(self):
        # this happens before any module has run, or after another planner module has failed.
        # assume MoveBase is running, no active plans
        # performance metrics: a-priori esitmates
        # isNewGoal = self.updateGoal()
        if self.currentActionGoal is None:
            return
        if max(self.performanceMetrics)>=1.0:
            print("INFO: Module", self.name, "ES trying to reset Performance Metric, was:", self.performanceMetrics, ".")
            # this module has failed previously. Check if the goal is in a new position.
            # if the goal is at a new position, reset retries = 0, availability to 1 (metric=0)
            if self.isNewGoal:
                self.isNewGoal = False
                hasFailedBefore = self.poseHasFailedBefore(compareCurrentPose=True, compareGoalPose=False)
                if hasFailedBefore:
                    print("INFO: Goal", self.currentActionGoal.move_base_goal, "has failed before for Module", self.name, ".")
                    # do nothing if the goal was retried multiple times without succeeding
                    return
                #Goal at a new position. reset retry counter. Also set self.resourceMetrics[0] = 0.0
                self.retries = 0
            elif self.retries >= self.max_retry:
                print("INFO: Goal", self.currentActionGoal.move_base_goal, "exceeded max retries for Module", self.name, ".")
                # goal at old position and max_retry exceeded. do nothing.
                return
            if self.performanceMetrics[0] >= 1.0:
                self.performanceMetrics[0] = self.defaultPerformanceMetrics[0]
            if self.performanceMetrics[1] >= 1.0:
                self.performanceMetrics[1] = self.defaultPerformanceMetrics[1]
            #if self.performanceMetrics[2] >= 1.0:
            #    self.performanceMetrics[2] = 0.0
        # configure the planner to what this module will have. This tests & updates availability metric.
        self.prelaunch()
        # this module is available and has not previously failed. test it.
        if self.resourceMetrics[0] < 1.0:
            print("INFO: Module", self.name, "ES determines resource available:", self.resourceMetrics, ".")
            try:
                # move_base/make_plan service available and generates a non-empty plan
                rospy.wait_for_service('move_base/make_plan', timeout=10.0)
            except rospy.ROSException as e:
                # service is unavailable. Notify.
                print("ERROR: Service unavailable: %s"%e)
                self.setInfeasible(self.resourceMetrics, 0)
                return
            start = PoseStamped()
            startTmp = self.getPoseInGoalFrame()
            start.header = startTmp.header
            start.pose = startTmp.pose.pose
            goal = PoseStamped(header = self.currentActionGoal.header,
                                pose = self.currentActionGoal.target_pose.pose)
            try:
                planSrv = rospy.ServiceProxy('move_base/make_plan', GetPlan)
                plan = planSrv(start, goal, self.currentActionGoal.goal_tolerance[0][0])
                self.resourceMetrics[0] = 0.0 if len(plan.plan.poses) else 1.0
                self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
            except rospy.ServiceException as e:
                print("ERROR: Service call failed: %s"%e)
                self.setInfeasible(self.resourceMetrics, 0)
        else:
            print("INFO: Module", self.name, "ES determines resource UNAVAILABLE:", self.resourceMetrics, ".")
