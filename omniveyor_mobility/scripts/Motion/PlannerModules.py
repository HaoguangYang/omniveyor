#!/usr/bin/python

import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from ReFrESH_ROS import ReFrESH_Module
from ReFrESH_ROS_utils import Ftype
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseGoal

""" Manipulate Move Base planners """
class moveBaseModule(ReFrESH_Module):
    def __init__(self, name="moveBaseMotion", priority=97, preemptive=True):
        super().__init__(name, priority=priority, preemptive=preemptive)
        # Tortuosity of path, time used, final error
        self.performanceMetrics = [0.0, 0.0, 0.0]
        # Resource metric: not-availability
        self.resourceMetrics = [0.5]
        # The class instance is mutable (call by reference)
        self.goal = MoveBaseGoal()
        self.setComponentProperties('EX', Ftype.ACTION_CLI, 'move_base', self.feedbackCb, mType=MoveBaseAction, 
                                    kwargs={'active_cb': self.activeCb, 'done_cb': self.doneCb, 'goal': self.goal,
                                            'prelaunch_cb': self.prelaunch, 'availTimeout': 1.0},
                                    ind=0)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0})
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)

    def prelaunch(self):
        # use dynamic reconfiguration to change the planner type to desired ones.
        pass
    
    def feedbackCb(self):
        # synchronize feedback to this class, and to the manager class.
        pass
    
    def activeCb(self):
        # notify the manager class that the goal has been accepted.
        pass
    
    def doneCb(self):
        # ring the bell on the manager about the result.
        pass

    def evaluator(self, event):
        # /move_base/make_plan service available, distance to goal, time elapsed, turtuosity of path
        # time_elapsed resets if distance_to_goal decreases every, say, 0.5m?
        # if timed out && distance > tolerance set failed flag.
        # if failed retries ++
        pass

    def estimator(self):
        # /move_base/make_plan service available and generates a non-empty plan
        # if retries < max_retry set availability to 1 (metric=0)
        # if retries >= max_retry and || position - last_failed_position || < 1m set availability to 0 (metric=1)
        # if || position - last_failed_position || > 1m set retries = 0, availability to 1 (metric=0)
        pass
    
"""TODO: Follows a path as-is with PID controller"""
class viaPointFollowerModule(ReFrESH_Module):
    def __init__(self, name="moveBaseMotion", priority=96, preemptive=True, pathTopic="raw_path"):
        super().__init__(name, priority=priority, preemptive=preemptive)
        # Tortuosity of path, time used, final error
        self.performanceMetrics = [0.0, 0.0, 0.0]
        # Resource metric: nearest obstacle threshold, CPU utilization, mem Utilization
        self.resourceMetrics = [0.5, 0.0, 0.0]
        # use two EX threads instead?
        self.setComponentProperties('EX', Ftype.SUBSCRIBER, pathTopic, self.getPath, mType=Path)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0})
        self.setComponentProperties('ES', Ftype.TIMER, exec=self.estimator, kwargs={'freq': 3.0})

    def getPath(self, msg):
        # pid controller to next point in the path
        pass

    def evaluator(self, event):
        # footprint intrusion
        pass

    def estimator(self, event):
        # is path clear, or, /move_base/make_plan service available and generates a non-empty plan?
        pass