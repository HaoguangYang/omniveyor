#!/usr/bin/python

import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import rospy
from ReFrESH_ROS import ReFrESH_Module
from ReFrESH_ROS_utils import Ftype, ROSnodeMonitor, ROSTopicMonitor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from Motion.utils import covToTolerance
import rospkg
from sensor_msgs.msg import Image, PointCloud2

class MarkerLocalizerModule(ReFrESH_Module):
    def __init__(self, name="markerLocalizer", priority=80, preemptive=False,
                EX_thread=1, EV_thread=1, ES_thread=1):
        """ module EX input: camera rgb image, camera info (intrinsics)
            module EX output: list of ArUCO markers and their raw poses
        """
        super().__init__(name, priority, preemptive,
                        EX_thread=EX_thread, EV_thread=EV_thread, ES_thread=ES_thread)
        self.exMon = ROSnodeMonitor()
        self.topicMon = ROSTopicMonitor(subs=[['cam_d1/color/image_raw', Image]])
        # CPU, memory, Topic and TF availability
        self.resourceMetrics = [0.1, 0.1, 0.0]
        # linear and angular stdev, 1-fulfillment
        self.performanceMetrics = [0.1, 0.1, 0.1]
        self.cpuQuota = 0.2
        self.memQuota = 0.2
        self.localizationTol = [0.05, 0.05]
        self.setComponentProperties('EX', Ftype.LAUNCH_FILE, 'omniveyor_mobility', 'aruco_detect.launch')
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0}, ind=0,
                                        pre=lambda : self.exMon.attach(self.getMyEXhandle()[1]),
                                        post=lambda : self.exMon.detach())
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)

    def setInfeasible(self, which, ind):
        which[ind] = 1.0
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

    def evaluator(self, event):
        """check CPU and Memory usage, check dependent topics & TF, check covariance of Map->Base TF, save map periodically

        Args:
            event (_type_): Timer event, triggered periodically
        """
        # check if performance monitor is attached
        if self.exMon.isAttached():
            if not all(self.topicMon.subsHaveSources()):
                self.setInfeasible(self.resourceMetrics, 2)
                return
            #if not all(self.topicMon.tfFeasible()):
            #    self.setInfeasible(self.resourceMetrics, 2)
            #    return
            self.resourceMetrics[2] = 0.0
            # log worst case CPU usage of the launched exec.
            uCPU, uMem = self.exMon.getCpuMemUtil()
            uCPU /= self.cpuQuota
            uMem /= self.memQuota
            # exponential filter for worst case execution time / memory utilization
            self.resourceMetrics[0] = max(uCPU, self.resourceMetrics[0]*0.975)
            self.resourceMetrics[1] = max(uMem, self.resourceMetrics[1]*0.975)
            # report self.bottleNeck to the performance aspect of the module
            # bottleneck is already updated in the decider.
            # TODO: performance metric
            # self.performanceMetrics = ...
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
            if self.reconfigMetric.bottleNeck >= 1.0 or self.save_map_interval < 0.:
                return
        else:
            # attach resource monitor to the move base instance
            self.exMon.attach(self.getMyEXhandle()[1])

    def estimator(self):
        if self.exMon.isAttached():
            self.exMon.detach()
        # check dependent topics & TF
        if not all(self.topicMon.subsHaveSources()):
            self.setInfeasible(self.resourceMetrics, 2)
            return
        #if not all(self.topicMon.tfFeasible()):
        #    self.setInfeasible(self.resourceMetrics, 2)
        #    return
        
        # TODO: if nearby the target pose, check if marker is visible by any cameras...

        self.resourceMetrics[2] = 0.0
        # clear cov of Map->Base TF
        self.performanceMetrics = [0.02, 0.02]
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

# TODO: below are still scratch

class ObjectLocalizerModule(ReFrESH_Module):
    def __init__(self, name="objectLocalizer", priority=80, preemptive-False,
                EX_thread=1, EV_thread=1, ES_thread=1):
        """ module EX input: camera rgb image, camera info (intrinsics), object reference (Image / CAD)
            module EX output: list of objects recognized, bbox list
        """

class DepthLookupModule(ReFrESH_Module):
    def __init__(self, name="depthLookup", priority=80, preemptive=False,
                EX_thread=1, EV_thread=1, ES_thread=1):
        """ module EX input: object size and pose, depth image, depth image info
            module EX output: object point cloud (X,Y,Z) in specified ref frame, point cloud center
        """
        super().__init__(name, priority, preemptive, 
                        EX_thread=EX_thread, EV_thread=EV_thread, ES_thread=ES_thread)

class GuessedDepthLookupModule(ReFrESH_Module):
    def __init__(self, name="guessedDepthLookup", priority=80, preemptive=False,
                EX_thread=1, EV_thread=1, ES_thread=1):
        """ module EX input: object size and pose, laser scan
            module EX output: object points (X,Y,Z) in specified ref frame, point center
        """
        super().__init__(name, priority, preemptive, 
                        EX_thread=EX_thread, EV_thread=EV_thread, ES_thread=ES_thread)
