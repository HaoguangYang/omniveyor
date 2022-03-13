#!/usr/bin/python
"""
Common ReFrESH Utilities
Author: Haoguang Yang
"""

import roslaunch
import rospy
import actionlib
from actionlib.action_server import nop_cb
from enum import Enum
import threading
import inspect
import ctypes
import psutil
import os
import netifaces as ni
import time

"""
Raise exception in a thread asynchronously.
"""
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    if not inspect.isclass(exctype):
        raise TypeError("Only types can be raised (not instances)")
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id:"+str(tid))
    elif res != 1:
        """if it returns a number greater than one, you're in trouble,
        and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, 0)
        raise SystemError("PyThreadState_SetAsyncExc failed")

"""
Thread class with stoppable exception.
"""
class Thread(threading.Thread):
    def _get_my_tid(self):
        """determines this (self's) thread id"""
        if not self.is_alive():
            # it may have shut down during this process.
            raise threading.ThreadError("the thread is not active")

        # do we have it cached?
        if hasattr(self, "_thread_id"):
            return self._thread_id

        # no, look for it in the _active dict
        for tid, tobj in threading._active.items():
            if tobj is self:
                self._thread_id = tid
                return tid

        raise AssertionError("could not determine the thread's id")

    def raise_exc(self, exctype):
        """raises the given exception type in the context of this thread"""
        _async_raise(self._get_my_tid(), exctype)

    def stop(self):
        """raises SystemExit in the context of the given thread, which should
        cause the thread to exit silently (unless caught)"""
        try:
            self.raise_exc(SystemExit)
        except (ValueError, threading.ThreadError) as e:
            print("WARNING: Thread.stop failed with", e, ". The thread may have died previously.")
        finally:
            self.join()

"""Class that implements a ring buffer"""
class RingBuffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self,size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """
        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max
        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self,x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

"""
Class that monitors CPU and memory utilization of ROS nodes that runs separately with the manager.
Supports nodes launched with launch file or with rosrun, within the Launcher class.
"""
class ROSnodeMonitor:
    def __init__(self):
        self.monitors = []

    def attach(self, handles):
        for launchThread in handles:
            if isinstance(launchThread, roslaunch.parent.ROSLaunchParent):
                self.monitors.append([psutil.Process(subProc.get_info()['pid']) for subProc in launchThread.pm.procs])
            elif isinstance(launchThread, roslaunch.Process):
                self.monitors.append([psutil.Process(launchThread.get_info()['pid'])])
            else:
                print("ERROR: Handle", launchThread, \
                        "is not of type Ftype.LAUNCH_FILE or Ftype.NODE. This thread is not supported.")
                self.monitors.append([])

    def getCpuMemUtil(self):
        cpuUtil = 0.
        memUtil = 0.
        for th in self.monitors:
            for this in th:
                cpuUtil += this.cpu_percent()*0.01
                memUtil += this.memory_percent()*0.01
        cpuUtil /= psutil.cpu_count()
        return cpuUtil, memUtil
    
    def detach(self):
        self.monitors = []
    
    def isAttached(self):
        return len(self.monitors)

"""Monitors network traffic flow of a wireless interface. Implementation based on iwconfig shell command"""
class WirelessNetworkMonitor:
    def __init__(self, hint='wl', interval=1.0):
        ifaces = ni.interfaces()
        self.wlan_name = ''
        for item in ifaces:
            if hint in item:
                self.wlan_name = item
                break
        if not self.wlan_name:
            raise RuntimeError("No WiFi interface is found under the given hint", hint, ".")
        self.dt = interval
        self.lastUpdate = time.time()
        self.maxBandwidth = 0.1
    
    def getInterfaceSpeed(self):
        currentTime = time.time()
        if currentTime - self.lastUpdate > self.dt:
            termOut = os.popen('iwconfig '+self.wlan_name+\
                               ' | grep "Mb/s" | sed "s/.*Bit Rate=\\([^ ]*\\) Mb.*/\\1/"').read()[:-1]
            if termOut:
                # convert to Bytes per second
                self.maxBandwidth = float(termOut)*131072
            else:
                # return a small value to prevent singularity
                self.maxBandwidth = 0.1
            self.lastUpdate = currentTime

    def bwUtil(self, bytesRecvd, dt):
        self.getInterfaceSpeed()
        return bytesRecvd/dt/self.maxBandwidth

"""
Thread Function Type enumerator
"""
class Ftype(Enum):
    NODE = 0
    LAUNCH_FILE = 1
    THREAD = 2
    TIMER = 3
    SUBSCRIBER = 4
    SERVICE = 5
    ACTION = 6

"""
Wrapper for launching threads and ROS components within Python
"""
class Launcher:
    def __init__(self, managerNodeName):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.roscoreProc = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], \
                                                            is_core=True)
        self.roscoreProc.start()
        rospy.init_node(managerNodeName)
        self.nodeLauncher = roslaunch.scriptapi.ROSLaunch()
        self.nodeLauncher.start()

    def launch(self, ftype, *args, **kwargs):
        if ftype == Ftype.NODE:
            return self.nodeLaunch(*args, **kwargs)
        elif ftype == Ftype.LAUNCH_FILE:
            return self.fileLaunch(*args, **kwargs)
        elif ftype == Ftype.THREAD:
            return self.threadLaunch(*args, **kwargs)
        elif ftype == Ftype.TIMER:
            return self.timerLaunch(*args, **kwargs)
        elif ftype == Ftype.SUBSCRIBER:
            return self.subscriberLaunch(*args, **kwargs)
        elif ftype == Ftype.SERVICE:
            return self.serviceLaunch(*args, **kwargs)
        elif ftype == Ftype.ACTION:
            return self.actionLaunch(*args, **kwargs)
        else:
            print("ERROR: type not implemented.")

    def nodeLaunch(self, pkgName, execName, name=None, namespace='/', args='', respawn=False, \
                 respawn_delay=0.0, remap_args=None, env_args=None, output=None, launch_prefix=None):
        try:
            nodeProc = self.nodeLauncher.launch(roslaunch.core.Node(pkgName, execName, name=name, \
                namespace=namespace, args=args, respawn=respawn, respawn_delay=respawn_delay, \
                remap_args=remap_args, env_args=env_args, output=output, launch_prefix=launch_prefix))
        except:
            nodeProc = None
        return nodeProc

    def fileLaunch(self, pkgName='', fileName='', fullPathList=[]):
        if not len(fullPathList):
            fullPathList = roslaunch.rlutil.resolve_launch_arguments([pkgName, fileName])
        launchProc = roslaunch.parent.ROSLaunchParent(self.uuid, fullPathList)
        launchProc.start()
        return launchProc

    def threadLaunch(self, funcPtr=None, args=None):
        t = Thread(target=funcPtr, args=args)
        t.start()
        # wait till it spins up.
        while not t.is_alive:
            pass
        return t

    def timerLaunch(self, freq, cb):
        if freq>0.:
            return rospy.Timer(rospy.Duration(1./freq), cb)
        else:
            print("ERROR: Initializing a timer callback with non-positive frequency.")

    def subscriberLaunch(self, topic, msgType, cb, args=None):
        return rospy.Subscriber(topic, msgType, cb, args)

    def serviceLaunch(self, topic, srvType, cb):
        return rospy.Service(topic, srvType, cb)

    def actionLaunch(self, topic, actType, cb, cancel_cb=nop_cb):
        actServer = actionlib.ActionServer(topic, actType, cb, cancel_cb, auto_start = False)
        actServer.start()
        return actServer

    def stop(self, proc):
        if callable(getattr(proc, "stop", None)):
            proc.stop()
        elif callable(getattr(proc, "shutdown", None)):
            proc.shutdown()
        elif callable(getattr(proc, "unregister", None)):
            proc.unregister()

    def shutdown(self):
        self.nodeLauncher.stop()
        self.roscoreProc.shutdown()

if __name__ == "__main__":
    pass
