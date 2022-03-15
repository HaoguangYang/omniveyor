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
        def clear(self):
            """ Clear and reset the buffer to not full """
            self.data.clear()
            self.__class__ = RingBuffer

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
    
    def clear(self):
        """ Clear and reset the buffer """
        self.data.clear()

"""
Class that monitors CPU and memory utilization of ROS nodes that runs separately with the manager.
Supports nodes launched with launch file or with rosrun, within the Launcher class.
"""
class ROSnodeMonitor:
    def __init__(self):
        self.monitors = []

    def attach(self, handles):
        try:
            for proc in handles:
                if hasattr(proc, '__iter__'):
                    for p in proc:
                        if isinstance(p, roslaunch.Process):
                            self.monitors.append([psutil.Process(p.get_info()['pid'])])
                        else:
                            print("ERROR: Handle", p, "within", proc, "is not supported.")
                elif isinstance(proc, roslaunch.Process):
                    self.monitors.append([psutil.Process(proc.get_info()['pid'])])
                else:
                    print("ERROR: Handle", proc, \
                            "is not of type Ftype.LAUNCH_FILE or Ftype.NODE. This thread is not supported.")
                    self.monitors.append([])
            for th in self.monitors:
                for this in th:
                    # initialize CPU counter
                    _ = this.cpu_percent()
        except psutil.NoSuchProcess:
            self.detach()

    def getCpuMemUtil(self):
        cpuUtil = 0.
        memUtil = 0.
        try:
            for th in self.monitors:
                for this in th:
                    cpuUtil += this.cpu_percent()*0.01
                    memUtil += this.memory_percent()*0.01
            cpuUtil /= psutil.cpu_count()
        except psutil.NoSuchProcess:
            self.detach()
        #print(cpuUtil, memUtil)
        return cpuUtil, memUtil
    
    def detach(self):
        self.monitors.clear()
    
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
        self.lastUpdate = 0
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
        #self.spinSet = set()
        self.roscoreProc = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], \
                                                            is_core=True)
        self.roscoreProc.start()
        #self.spinSet.add(self.roscoreProc)
        rospy.init_node(managerNodeName)
        self.nodeLauncher = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], \
                                                            is_core=False)
        self.nodeLauncher.start(auto_terminate=False)
        #self.spinSet.add(self.nodeLauncher)

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
        nodeProc, success = self.nodeLauncher.runner.launch_node(roslaunch.core.Node(\
                pkgName, execName, name=name, namespace=namespace, args=args, respawn=respawn, \
                respawn_delay=respawn_delay, remap_args=remap_args, env_args=env_args, \
                output=output, launch_prefix=launch_prefix))
        if success:
            return nodeProc
        else:
            return None

    def fileLaunch(self, pkgName='', fileName='', fullPathList=[]):
        if not len(fullPathList):
            fp = roslaunch.rlutil.resolve_launch_arguments([pkgName, fileName])
        else:
            fp = fullPathList
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        cfg = roslaunch.config.load_config_default(fp, None, verbose=False)
        nodeProcs = []
        # only launches local nodes.
        local_nodes = [n for n in cfg.nodes if roslaunch.core.is_machine_local(n.machine)]
        for node in local_nodes: 
            proc, success = self.nodeLauncher.runner.launch_node(node)
            if success:
                nodeProcs.append(proc)
        return tuple(nodeProcs)

    def threadLaunch(self, funcPtr=None, args=()):
        t = Thread(target=funcPtr, args=args)
        t.start()
        return t

    def timerLaunch(self, freq, cb):
        if freq>0.:
            timer = rospy.Timer(rospy.Duration(1./freq), cb)
            return timer
        else:
            print("ERROR: Initializing a timer callback with non-positive frequency.")

    def subscriberLaunch(self, topic, msgType, cb, args=None):
        subs = rospy.Subscriber(topic, msgType, cb, args)
        return subs

    def serviceLaunch(self, topic, srvType, cb):
        srv = rospy.Service(topic, srvType, cb)
        return srv

    def actionLaunch(self, topic, actType, cb, cancel_cb=nop_cb):
        actServer = actionlib.ActionServer(topic, actType, cb, cancel_cb, auto_start = False)
        actServer.start()
        return actServer

    def stop(self, proc):
        if hasattr(proc, '__iter__'):
            for p in proc:
                if callable(getattr(p, "stop", None)):
                    p.stop()
                elif callable(getattr(p, "shutdown", None)):
                    p.shutdown()
                elif callable(getattr(p, "unregister", None)):
                    p.unregister()
                else:
                    raise RuntimeError("Stopping method not implemented!")
        else:
            if callable(getattr(proc, "stop", None)):
                proc.stop()
            elif callable(getattr(proc, "shutdown", None)):
                proc.shutdown()
            elif callable(getattr(proc, "unregister", None)):
                proc.unregister()
            else:
                raise RuntimeError("Stopping method not implemented!")

    def spin(self):
        while self.roscoreProc.pm.procs:
            self.roscoreProc.spin_once()
            self.nodeLauncher.spin_once()
            time.sleep(0.1)
        print ("INFO: ROS Launcher Exiting...")
    
    def spin_once(self):
        self.nodeLauncher.spin_once()
        self.roscoreProc.spin_once()
        
    def shutdown(self):
        self.nodeLauncher.shutdown()
        self.roscoreProc.shutdown()

if __name__ == "__main__":
    pass