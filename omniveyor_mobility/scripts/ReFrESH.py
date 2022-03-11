#!/usr/bin/python
"""
ReFrESH module that runs on ROS
v0.2.1
Author: Haoguang Yang
Change Log:
v0.1        09/26/2021      First implementation based on SMACH
v0.2.0      03/09/2022      Refactoring the code to use without SMACH for better efficiency
v0.2.1      03/11/2022      Support multi-threaded EX, EV, ES.
                            Support prioritized tasks and preemption

Reconfiguration Framework for distributed Embedded systems on Software and Hardware
Originally designed to run on FPGA, the script brings self-adaptation to robots running Linux
and ROS. Specifically, the triplet EX/EV/ES are enforced in a functional module, with their
states managed by a performance- and resource-aware decider.

Reference for the original work:
Cui, Y., Voyles, R. M., Lane, J. T., Krishnamoorthy, A., & Mahoor, M. H. (2015). A mechanism
for real-time decision making and system maintenance for resource constrained robotic systems
through ReFrESH. Autonomous Robots, 39(4), 487-502.
"""

import roslaunch
import rospy
import actionlib
from actionlib.action_server import nop_cb
from enum import Enum
import threading
import inspect
import ctypes
import sys

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
        self.roscoreProc = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], is_core=True)
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
        if len(fullPathList):
            launchProc = roslaunch.parent.ROSLaunchParent(self.uuid, fullPathList)
            launchProc.start()
            return launchProc
        else:
            fullPath = roslaunch.rlutil.resolve_launch_arguments([pkgName, fileName])
            launchProc = roslaunch.parent.ROSLaunchParent(self.uuid, fullPath)
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

"""
Element of a thread in a ReFrESH module
"""
class ModuleComponent:
    def __init__(self, ftype, args, kwargs):
        self.ftype = ftype
        self.args = args
        self.kwargs = kwargs

"""
Metrics for deciding reconfiguration
"""
class ReconfigureMetric:
    def __init__(self):
        """
        normalized performance utilization: a continuous value between 0 and 1.
        1: performance degraded to the tolerable bounds.
        0: performance is the same as the ideal condition.
        """
        self.performanceUtil = 0.
        """
        normalized resource utilization: a continuous value between 0 and 1.
        1: the module utilizes all quota of available resources.
        0: the module utilizes no resource.
        """
        self.resourceUtil = 0.
    
    def update(self, performanceDims, resourceDims):
        self.performanceUtil = max(performanceDims)
        self.resourceUtil = max(resourceDims)

"""
The base class of a ReFrESH module
"""
class ReFrESH_Module:
    def __init__(self, name, priority=0, preemptive=False, EX_thread=1, EV_thread=1, ES_thread=1):
        self.name = name
        # the higher priority, the more important.
        self.priority = priority
        """preemptive: This module suspends all other modules in the enabled list
        that has lower priority, until finished"""
        self.preemptive = preemptive
        self.onHandle = None
        self.offHandle = None
        self.EX_thread = EX_thread
        self.EV_thread = EV_thread
        self.ES_thread = ES_thread
        """The unified reconfiguration metric is updated by the EV (module ON) or ES (module OFF)"""
        self.reconfigMetric = ReconfigureMetric()
        self.EX = [ModuleComponent(Ftype.THREAD, (), {}) for i in range(self.EX_thread)]
        self.EV = [ModuleComponent(Ftype.THREAD, (), {}) for i in range(self.EV_thread)]
        self.ES = [ModuleComponent(Ftype.THREAD, (), {}) for i in range(self.ES_thread)]
    
    """
    Helper function to set the property of a thread
    which:  either EX, EV, or ES
    ind:    thread index within the list (0..*_thread-1)
    ftype:  enumerated thread function type (Ftype)
    ns:     topic or package name, depending on the context. String
    exec:   function pointer
    args:   arguments of the function pointer. Tuple
    mType:  message/service/action type depending on the context
    kwargs: Keyword arguments. Dict
    """
    def setComponentProperties(self, which, ind, ftype, ns='', exec=None, args=None, mType=None, \
                                kwargs=None):
        this = None
        if (which in ['EX', 'ex', 'Execute', 'execute', 'Executor', 'executer', self.EX]):
            this = self.EX[ind]
        elif (which in ['EV', 'ev', 'Evaluate', 'evaluate', 'Evaluator', 'evaluator', self.EV]):
            this = self.EV[ind]
        elif (which in ['ES', 'es', 'Estimate', 'estimate', 'Estimator', 'estimator', self.ES]):
            this = self.ES[ind]
        else:
            raise TypeError("Invalid set-property request!")
        this.ftype = ftype
        if ftype == Ftype.NODE:
            this.kwargs = {'pkgName': ns, 'execName': exec, 'args': args}
            this.kwargs = {**this.kwargs, **kwargs}
        elif ftype == Ftype.LAUNCH_FILE:
            this.kwargs = {'pkgName': ns, 'fileName': exec}
            this.kwargs = {**this.kwargs, **kwargs}
        elif ftype == Ftype.THREAD:
            this.kwargs = {'funcPtr': exec, 'args': args}
        elif ftype == Ftype.TIMER:
            this.kwargs = {'cb': exec}
            this.kwargs = {**this.kwargs, **kwargs}
        elif ftype == Ftype.SUBSCRIBER:
            this.kwargs = {'topic': ns, 'msgType': mType, 'cb': exec, 'args': args}
        elif ftype == Ftype.SERVICE:
            this.kwargs = {'topic': ns, 'srvType': mType, 'cb': exec}
        elif ftype == Ftype.ACTION:
            this.kwargs = {'topic': ns, 'actType': mType, 'cb': exec}
            this.kwargs = {**this.kwargs, **kwargs}
        else:
            print("ERROR: type not implemented.")
    
    """Helper function to turn on the module from within"""
    def turnOn(self):
        if callable(self.onHandle):
            self.onHandle(self)
        else:
            print("ERROR: turnOn for this module is not registered to a manager.")
    
    """Helper function to turn off the module from within"""
    def turnOff(self):
        if callable(self.offHandle):
            self.offHandle(self)
        else:
            print("ERROR: turnOff for this module is not registered to a manager.")

"""
Base class of a module manager and decider.
Manages a given set of modules, documenting their states being either off, ready (preempted), or on.
Turn on or off the modules based on request, or through self adaptation using the basicDecider.
The basicDecider turns off a module as either its performance or resource utilization exceeds 1.
The basicDecider then search within its managed module to start the one with minimum utilization,
taking the larger one of the two aspects.
"""
class Manager:
    def __init__(self, launcher, managedModules=[], name="", freq=5.0):
        self.name = name
        self.launcher = None
        if isinstance(launcher, Launcher):
            self.launcher = launcher
        else:
            raise TypeError("Not initializing the manager with the correct ROS launcher.")
        # No memory copy
        self.moduleSet = set(item for item in managedModules)   # make it a set of class pointers
        self.onSet = set()      # set of triplets: (name, (ex), (ev))
        self.readySet = set()   # set of ready (preempted) modules: name
        self.offSet = set()     # set of duets: (name, (es))
        self.Decider = ModuleComponent(Ftype.TIMER, (freq, self.basicDecider), {})
        self.Decider_proc = None
        for m in managedModules:
            if isinstance(m, ReFrESH_Module):
                # register handler
                m.onHandle = self.turnOn
                m.offHandle = self.turnOff
                # turn on ES
                es = (self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) for th in m.ES)
                self.offSet.add((m.name, es))
                print("INFO: Module", m.name, "SPAWNED with Manager", self.name, ".")
            else:
                raise TypeError("Not initializing Manager", self.name, \
                                "with the supported module class: ReFrESH_Module.")
    
    def moduleIsOn(self, module):
        res = tuple(m for m in self.onSet if m[0]==module.name)
        return len(res), res
    
    def moduleIsOff(self, module):
        res = tuple(m for m in self.offSet if m[0]==module.name)
        return len(res), res
    
    def turnOn(self, module):
        if module in self.moduleSet:
            isOn, _ = self.moduleIsOn(module)
            if isOn:
                # module is already on
                print("ERROR: ON request for module", module.name, ", which is already ON.")
            else:
                for m in self.moduleSet:
                    if m != module and m.priority > module.priority:
                        isOn, _ = self.moduleIsOn(m)
                        if isOn and m.preemptive:
                            # a preemptive task with higher priority is running. add to ready set instead.
                            self.readySet.add(module.name)
                            return
                # we are safe to start this module.
                # turn off ES
                isOff, res_off = self.moduleIsOff(module)
                if isOff:
                    for item in res_off:
                        for th in item[1]:
                            self.launcher.stop(th)
                        self.offSet.remove(item)
                else:
                    raise RuntimeError("The managed module is neither ON nor OFF. Something bad happened.")
                # turn on EX
                ex = (self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                        for th in module.EX)
                # turn on EV
                ev = (self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                        for th in module.EV)
                # add (module, proc) to self.onSet
                self.onSet.add((module.name, ex, ev))
                # Is this module recovered from preemption?
                if module.name in self.readySet:
                    self.readySet.remove(module.name)
                    print("INFO: Module", module.name, "ON from preemption.")
                else:
                    print("INFO: Module", module.name, "ON.")
                # this module preempts all other modules with lower priority
                if module.preemptive:
                    for m in self.moduleSet:
                        if m != module and m.priority <= module.priority:
                            isOn, _ = self.moduleIsOn(m)
                            if isOn:
                                self.turnOff(m)
                                self.readySet.add(m.name)
                                print("INFO: Module", module.name, "preempted", m.name)
        else:
            print("ERROR: Module", module, "(name:", module.name, ") is not managed by this instance.")
    
    def turnOff(self, module):
        if module in self.moduleSet:
            isOn, res = self.moduleIsOn(module)
            if isOn:
                for item in res:
                    # turn off EX
                    for th in item[1]:
                        self.launcher.stop(th)
                    # turn off EV
                    for th in item[2]:
                        self.launcher.stop(th)
                    # remove res from self.onSet
                    self.onSet.remove(item)
                isOff, _ = self.moduleIsOff(module)
                if isOff:
                    raise RuntimeError("The managed module is both ON and OFF. Something bad happened.")
                else:
                    # turn on ES
                    es = (self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                            for th in module.ES)
                    self.offSet.add((module.name, es))
                print("INFO: Module", module.name, "OFF.")
                # we have nothing running... Is there anything previously preempted?
                if (not len(self.onSet)) and len(self.readySet):
                    prio = - sys.maxsize - 1
                    nextOn = None
                    for m in self.moduleSet:
                        if m.name in self.readySet:
                            if m.priority > prio:
                                prio = m.priority
                                nextOn = m
                    if nextOn:
                        # turn on m
                        self.turnOn(nextOn) 
            else:
                if module.name in self.readySet:
                    self.readySet.remove(module.name)
                    print("INFO: Module", module.name, "OFF after preemption.")
                else:
                    print("ERROR: OFF request for module", module.name, ", which is already OFF.")
        else:
            print("ERROR: Module", module, "(name:", module.name, ") is not managed by this instance.")
    
    def basicDecider(self, event):
        for module in self.moduleSet:
            isOn, _ = self.moduleIsOn(module)
            if isOn:
                if (module.reconfigMetric.performanceUtil > 1. or module.reconfigMetric.resourceUtil > 1.):
                    # performance crisis. do reconfig
                    print("INFO: Module", module.name, "falls below desired performance bounds.")
                    candidate = module
                    bestPerf = 1.
                    for m in self.moduleSet:
                        mIsOn, _ = self.moduleIsOn(m)
                        if m != module and not mIsOn:
                            tmp = max(m.reconfigMetric.performanceUtil, m.reconfigMetric.resourceUtil)
                            if tmp < bestPerf:
                                candidate = m
                                bestPerf = tmp
                    if candidate != module:
                        print("INFO: Found alternative module", candidate.name, "with estimated performance (", \
                            m.reconfigMetric.performanceUtil, m.reconfigMetric.resourceUtil, ").")
                        self.turnOn(candidate)
                        self.turnOff(module)
                        break
                    else:
                        print("WARN: No alternative module satisfies resource and performance bounds. Current value: (", \
                            m.reconfigMetric.performanceUtil, m.reconfigMetric.resourceUtil, ").")
    
    def requestOn(self, nameList):
        for module in self.moduleSet:
            isOn, _ = self.moduleIsOn(module)
            if not isOn and module.name in nameList:
                self.turnOn(module)
    
    def requestOff(self, nameList):
        for module in self.moduleSet:
            isOff, _ = self.moduleIsOff(module)
            if not isOff and module.name in nameList:
                self.turnOff(module)
    
    """This function blocks after starting the Decider, until ROS shutsdown"""
    def run(self):
        self.Decider_proc = self.launcher.launch(self.Decider.ftype, \
                                                *tuple(self.Decider.args), \
                                                **dict(self.Decider.kwargs))
        print("INFO: Launched decider thread.")
        rospy.spin()
    
    """This function returns immediately after starting the Decider."""
    def run_nonblock(self):
        self.Decider_proc = self.launcher.launch(self.Decider.ftype, \
                                                *tuple(self.Decider.args), \
                                                **dict(self.Decider.kwargs))
        print("INFO: Launched decider thread.")
    
    def shutdown(self):
        for m in self.onSet:
            # turn off EX
            for th in m[1]:
                self.launcher.stop(th)
            # turn off EV
            for th in m[2]:
                self.launcher.stop(th)
            print("INFO: Module", m[0], "SHUTDOWN.")
        for m in self.offSet:
            # turn off ES
            for th in m[1]:
                self.launcher.stop(th)
            print("INFO: Module", m[0], "SHUTDOWN.")

if __name__ == "__main__":
    # a simple test case with two empty modules.
    taskLauncher = Launcher("RobotTaskRunner")
    testModule1 = ReFrESH_Module("test1", priority=10, preemptive=True)
    testModule2 = ReFrESH_Module("test2")
    taskManager = Manager(taskLauncher, [testModule1, testModule2])
    # non-blocking run
    taskManager.run_nonblock()
    # turn on by requesting the manager
    taskManager.requestOn(["test1", "test2"])
    rospy.sleep(rospy.Duration(1.0))
    # turn on/off by requesting the module
    testModule1.turnOff()
    rospy.sleep(rospy.Duration(1.0))
    testModule2.turnOff()
    rospy.sleep(rospy.Duration(1.0))
    # respawnable
    testModule1.turnOn()
    rospy.sleep(rospy.Duration(1.0))
    # simulate a situation with degrading module
    testModule1.reconfigMetric.update([1.1], [1.1])
    rospy.spin()
    # shutdown
    taskManager.shutdown()
    taskLauncher.shutdown()