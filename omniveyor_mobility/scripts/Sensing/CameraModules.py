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
from slam_toolbox_msgs.srv import SaveMap, SerializePoseGraph
import rospkg
from sensor_msgs.msg import Image, PointCloud2

class MarkerLocalizerModule(ReFrESH_Module):
    def __init__(self, name="markerLocalizer", priority=80, preemptive=False,
                EX_thread=1, EV_thread=1, ES_thread=1):
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
        self.setComponentProperties('EX', Ftype.NODE, 'omniveyor_mobility', 'marker_localizer')
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
            if not all(self.topicMon.tfFeasible()):
                self.setInfeasible(self.resourceMetrics, 2)
                return
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
            locStdev = covToTolerance(self.slamLocCov.data)
            self.performanceMetrics = [locStdev[0]/self.localizationTol[0], locStdev[3]/self.localizationTol[1]]
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
            if self.reconfigMetric.bottleNeck >= 1.0 or self.save_map_interval < 0.:
                return
            timeNow = rospy.Time.now()
            if (timeNow-self.lastMapSavingTime).to_sec() < self.save_map_interval:
                return
            # Save Posegraph, PGM maps
            try:
                mapSaver = rospy.ServiceProxy('slam_toolbox/serialize_map', SerializePoseGraph)
                mapSaver(self.map_file_name)
            except rospy.ServiceException as e:
                print("ERROR: Service call failed: %s"%e)
            try:
                mapSaver = rospy.ServiceProxy('slam_toolbox/save_map', SaveMap)
                mapSaver(self.map_file_name)
            except rospy.ServiceException as e:
                print("ERROR: Service call failed: %s"%e)
            self.lastMapSavingTime = timeNow
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
        if not all(self.topicMon.tfFeasible()):
            self.setInfeasible(self.resourceMetrics, 2)
            return
        self.resourceMetrics[2] = 0.0
        # clear cov of Map->Base TF
        self.performanceMetrics = [0.02, 0.02]
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

class Static2DMapModule(ReFrESH_Module):
    def __init__(self, name="dynamicMap", priority=89, preemptive=False,
                EX_thread=1, EV_thread=1, ES_thread=1, map_file_name="map"):
        super().__init__(name, priority, preemptive, 
                        EX_thread=EX_thread, EV_thread=EV_thread, ES_thread=ES_thread)
        self.map_file_name = rospkg.RosPack().get_path('omniveyor_mobility')+'/resources/maps'+map_file_name
        self.exMon = ROSnodeMonitor()
        self.topicMon = ROSTopicMonitor(subs=[['scan', LaserScan]], tf=[['laser','odom']],
                                        tfBuff=self.managerHandle.tfBuffer)
        # CPU, memory, Topic and TF availability, FileExists
        self.resourceMetrics = [0.1, 0.1, 0.0, 0.0]
        # linear and angular stdev
        self.performanceMetrics = [0.1, 0.1]
        self.mapTimeStamp = 0.0
        self.setComponentProperties('EX', Ftype.LAUNCH_FILE, 'omniveyor_mobility', 'static_map.launch')
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0},
                                        pre=lambda : self.exMon.attach(self.getMyEXhandle()[1]),
                                        post=lambda : self.exMon.detach())
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)

    def setInfeasible(self, which, ind):
        which[ind] = 1.0
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

    def evaluator(self, event):
        """check CPU and Memory usage, check dependent topics & TF, check covariance of Map->Base TF

        Args:
            event (_type_): Timer event, triggered periodically
        """
        # check if performance monitor is attached
        if self.exMon.isAttached():
            if not all(self.topicMon.subsHaveSources()):
                self.setInfeasible(self.resourceMetrics, 2)
                return
            if not all(self.topicMon.tfFeasible()):
                self.setInfeasible(self.resourceMetrics, 2)
                return
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
            locStdev = covToTolerance(self.slamLocCov.data)
            self.performanceMetrics = [locStdev[0]/self.localizationTol[0], locStdev[3]/self.localizationTol[1]]
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
            if self.reconfigMetric.bottleNeck >= 1.0 or self.save_map_interval < 0.:
                return
        else:
            # attach resource monitor to the move base instance
            self.exMon.attach(self.getMyEXhandle()[1])

    def estimator(self):
        if self.exMon.isAttached():
            self.exMon.detach()
        if not os.path.exists(self.map_file_name+'.posegraph'):
            self.setInfeasible(self.resourceMetrics, 3)
            return
        if not os.path.exists(self.map_file_name+'.data'):
            self.setInfeasible(self.resourceMetrics, 3)
            return
        mapTime = os.path.getmtime(self.map_file_name+'.posegraph')
        if mapTime > self.mapTimeStamp:
            # map has been modified since last query. clear old stdev
            self.performanceMetrics = [0.0, 0.0]
            self.mapTimeStamp = mapTime
        self.resourceMetrics[3] = 0.0
        if not all(self.topicMon.subsHaveSources()):
            self.setInfeasible(self.resourceMetrics, 2)
            return
        if not all(self.topicMon.tfFeasible()):
            self.setInfeasible(self.resourceMetrics, 2)
            return
        self.resourceMetrics[2] = 0.0
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
