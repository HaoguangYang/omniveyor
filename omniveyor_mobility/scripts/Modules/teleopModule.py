#!/usr/bin/python

import rospy
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import glob
import socket
import select
import time
from ReFrESH import ReFrESH_Module
from ReFrESH_utils import Ftype, ROSnodeMonitor, WirelessNetworkMonitor
from geometry_msgs.msg import Twist

"""Take teleoperation input from joystick"""
class joystickTeleopModule(ReFrESH_Module):
    def __init__(self, name="joystickTeleop", priority=99, preemptive=True):
        super().__init__(name, priority=priority, preemptive=preemptive)
        # always ideal
        self.performanceMetrics = [0.0]
        # Resource metric: availability, CPU time, memory
        self.resourceMetrics = [0.0, 0.0, 0.0]
        self.cpuQuota = 0.2
        self.memQuota = 0.1
        self.exMon = ROSnodeMonitor()
        self.setComponentProperties('EX', Ftype.NODE, 'pcv_base', 'teleop_twist_joystick.py')
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 1.0})
        self.setComponentProperties('ES', Ftype.TIMER, exec=self.estimator, kwargs={'freq': 1.0})

    def hasJoystick(self):
        inputList = glob.glob('/dev/input/js*')
        if len(inputList):
            return inputList[0]
        else:
            return None

    def evaluator(self, event):
        # check if performance monitor is attached
        if self.exMon.isAttached():
            self.resourceMetrics[0] = 0.0 if self.hasJoystick() else 1.0
            # log worst case CPU usage of the launched exec.
            uCPU, uMem = self.exMon.getCpuMemUtil()
            uCPU /= self.cpuQuota
            uMem /= self.memQuota
            # exponential filter for worst case execution time / memory utilization
            self.resourceMetrics[1] = max(uCPU, self.resourceMetrics[1]*0.975)
            self.resourceMetrics[2] = max(uMem, self.resourceMetrics[2]*0.975)
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
        else:
            # attach performance monitor for the roslaunch process (EX)
            self.exMon.attach(self.getEXhandle())

    def estimator(self, event):
        # need to clean up EX monitor since it is inactive
        if self.exMon.isAttached():
            self.exMon.detach()
        self.resourceMetrics[0] = 0.0 if self.hasJoystick() else 1.0
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

"""Take teleoperation input from a received topic"""
class remoteTeleopModule(ReFrESH_Module):
    def __init__(self, name="remoteTeleop", priority=98, preemptive=True, rxPort=17102, namespace='remote_teleop'):
        super().__init__(name, priority=priority, preemptive=preemptive)
        self.rxPort = rxPort
        self.namespace = namespace

        # Performance metric: communication dropout tolerance (s)
        self.performanceMetrics = [0.0]
        self.commOutTol = 1.0
        self.lastCommRecvd = time.time()
        
        # Resource metric: bandwidth, CPU time, memory
        self.resourceMetrics = [0.0, 0.0, 0.0]
        self.bandwidthQuota = 0.2
        self.cpuQuota = 0.2
        self.memQuota = 0.1
        self.exMon = ROSnodeMonitor()
        self.netMon = WirelessNetworkMonitor()

        self.setComponentProperties('EX', Ftype.LAUNCH_FILE, 'pcv_base', 'remote_teleop.launch')
        self.setComponentProperties('EV', Ftype.SUBSCRIBER, 'cmd_vel', self.evaluator, mType=Twist)
        self.setComponentProperties('ES', Ftype.THREAD, exec=self.estimator)

    def evaluator(self, msg):
        # check if performance monitor is attached
        if self.exMon.isAttached():
            # log time difference since last message
            timeNow = time.time()
            dt = timeNow-self.lastCommRecvd
            self.lastCommRecvd = timeNow
            self.performanceMetrics[0] = dt/self.commOutTol
            # log network speed and utilization
            dataSize = sys.getsizeof(msg)
            self.resourceMetrics[0] = self.netMon.bwUtil(dataSize, dt)/self.bandwidthQuota
            # log worst case CPU usage of the launched exec.
            uCPU, uMem = self.exMon.getCpuMemUtil()
            uCPU /= self.cpuQuota
            uMem /= self.memQuota
            # exponential filter for worst case execution time / memory utilization
            self.resourceMetrics[1] = max(uCPU, self.resourceMetrics[1]*0.975)
            self.resourceMetrics[2] = max(uMem, self.resourceMetrics[2]*0.975)
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
        else:
            # attach performance monitor for the roslaunch process (EX)
            self.exMon.attach(self.getEXhandle())

    def estimator(self):
        # need to clean up EX monitor since it is inactive
        if self.exMon.isAttached():
            self.exMon.detach()
        # create a UDP socket to listen the remote control command port for incoming
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
        # shared port with other processes
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(0)
        try:
            sock.bind(('0.0.0.0', self.rxPort))
            while not rospy.is_shutdown():
                select.select([sock], [], [], self.commOutTol)   # blocks here
                try:
                    dat, _ = sock.recvfrom(131072)
                except socket.error:
                    pass
                else:
                    timeNow = time.time()
                    dataSize = len(dat)
                    dt = timeNow-self.lastCommRecvd
                    if dataSize:
                        self.lastCommRecvd = timeNow
                        self.resourceMetrics[0] = self.netMon.bwUtil(dataSize, dt)/self.bandwidthQuota
                        self.performanceMetrics[0] = dt/self.commOutTol
                    self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
                    time.sleep(0.1)
        except SystemExit:
            # this module is turned on.
            sock.close()
            # if this module is turned on, refresh the last recv time to avoid jump
            self.lastCommRecvd = time.time()

"""Take teleoperation input from keyboard
this is only a plan-B for teleoperation, since the keyboard detection is not robust,
and the console can be used for other purposes."""
class keyboardTeleopModule(ReFrESH_Module):
    def __init__(self, name="keyboardTeleop", priority=97, preemptive=True):
        super().__init__(name, priority=priority, preemptive=preemptive)
        # always ideal
        self.performanceMetrics = [0.0]
        # Resource metric: availability, CPU time, memory
        self.resourceMetrics = [0.0, 0.0, 0.0]
        self.cpuQuota = 0.2
        self.memQuota = 0.1
        self.exMon = ROSnodeMonitor()
        self.setComponentProperties('EX', Ftype.NODE, 'pcv_base', 'teleop_twist_keyboard.py')
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 1.0})
        self.setComponentProperties('ES', Ftype.TIMER, exec=self.estimator, kwargs={'freq': 1.0})

    def hasKeyboard(self):
        # get the last input device in the list, if the list is not empty.
        return os.popen('ls /dev/input/by-id | grep "kbd" | tail -1').read()[:-1]

    def hasSSH(self):
        # get bool of whether an ssh session is established.
        env = os.environ
        return ('SSH_CLIENT' in env or 'SSH_TTY' in env or 'SSH_CONNECTION' in env)

    def evaluator(self, event):
        # check if performance monitor is attached
        if self.exMon.isAttached():
            self.resourceMetrics[0] = 0.0 if len(self.hasKeyboard()) or self.hasSSH() else 1.0
            # log worst case CPU usage of the launched exec.
            uCPU, uMem = self.exMon.getCpuMemUtil()
            uCPU /= self.cpuQuota
            uMem /= self.memQuota
            # exponential filter for worst case execution time / memory utilization
            self.resourceMetrics[1] = max(uCPU, self.resourceMetrics[1]*0.975)
            self.resourceMetrics[2] = max(uMem, self.resourceMetrics[2]*0.975)
            self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)
        else:
            # attach performance monitor for the roslaunch process (EX)
            self.exMon.attach(self.getEXhandle())

    def estimator(self, event):
        # need to clean up EX monitor since it is inactive
        if self.exMon.isAttached():
            self.exMon.detach()
        self.resourceMetrics[0] = 0.0 if len(self.hasKeyboard()) or self.hasSSH() else 1.0
        self.reconfigMetric.update(self.performanceMetrics, self.resourceMetrics)

if __name__ == "__main__":
    from ReFrESH_utils import Launcher
    from ReFrESH import Manager
    # a simple test case with three teleop modules.
    taskLauncher = Launcher("teleopManager")
    jsMod = joystickTeleopModule()
    rmMod = remoteTeleopModule()
    kbMod = keyboardTeleopModule()
    taskManager = Manager(taskLauncher, [jsMod, rmMod, kbMod])
    # non-blocking run
    taskManager.run_nonblock()
    # turn on by requesting the manager
    taskManager.requestOn(["remoteTeleop", "joystickTeleop"])
    rospy.sleep(rospy.Duration(1.0))
    # turn on/off by requesting the module
    jsMod.turnOff()
    rospy.sleep(rospy.Duration(1.0))
    kbMod.turnOn()
    rospy.sleep(rospy.Duration(1.0))
    rmMod.turnOff()
    rospy.sleep(rospy.Duration(1.0))
    rospy.spin()
    # shutdown
    taskManager.shutdown()
    taskLauncher.shutdown()