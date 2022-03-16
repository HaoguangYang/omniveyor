#!/usr/bin/python
"""
ReFrESH module that runs on ROS
v0.2.1
Author: Haoguang Yang
Change Log:
v0.1        09/26/2021      First implementation based on SMACH.
v0.2.0      03/09/2022      Refactoring the code to use without SMACH for better efficiency.
v0.2.1      03/11/2022      Support multi-threaded EX, EV, ES. Support normalization within 
                            reconfig metric class. Support prioritized tasks and preemption.
v0.2.2      03/13/2022      Separated utilities to a separate script. Added handle to access
                            manager and EX/EV/ES processes within the module. Added WiFi and
                            CPU/Memory utilization monitors for modules.

Reconfiguration Framework for distributed Embedded systems on Software and Hardware
Originally designed to run on FPGA, the script brings self-adaptation to robots running Linux
and ROS. Specifically, the triplet EX/EV/ES are enforced in a functional module, with their
states managed by a performance- and resource-aware decider.

Reference for the original work:
Cui, Y., Voyles, R. M., Lane, J. T., Krishnamoorthy, A., & Mahoor, M. H. (2015). A mechanism
for real-time decision making and system maintenance for resource constrained robotic systems
through ReFrESH. Autonomous Robots, 39(4), 487-502.
"""

import rospy
import sys
import numpy as np
from ReFrESH_ROS_utils import *

"""
Element of a thread in a ReFrESH module
"""
class ModuleComponent:
    def __init__(self, ftype, args, kwargs):
        self.ftype = ftype
        self.args = args
        self.kwargs = kwargs

"""
Basic metrics class for deciding reconfiguration
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
        1: the module utilizes all quota of available resources / resource Not Available.
        0: the module utilizes no resource / resource Available.
        """
        self.resourceUtil = 0.
        """
        raw limits, following the performanceDims and resourceDims order
        """
        self.performanceDenominator = None
        self.resourceDenominator = None

    """ Set limits for normalizing the performance metric. """
    def setLimits(self, performanceLim, resourceLim):
        self.performanceDenominator = np.array(performanceLim)
        self.resourceDenominator = np.array(resourceLim)

    """ Normalize performance metric vector and update record. """
    def updateFromRaw(self, performanceDims, resourceDims):
        if self.performanceDenominator and self.resourceDenominator:
            self.performanceUtil = max((np.array(performanceDims)/self.performanceDenominator).tolist())
            self.resourceUtil = max((np.array(resourceDims)/self.resourceDenominator).tolist())
        else:
            print("ERROR: Limits not set. Metrics are not updated.")

    """ Update normalized metrics directly. """
    def update(self, performanceDims=None, resourceDims=None):
        if performanceDims:
            self.performanceUtil = max(performanceDims)
        if resourceDims:
            self.resourceUtil = max(resourceDims)

    """ for the base class, assume the bottleneck is the max among
    normalized performance and resource factors """
    def bottleNeck(self):
        return max([self.performanceUtil, self.resourceUtil])

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
        self.managerHandle = None
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
    ftype:  enumerated thread function type (Ftype)
    ns:     topic or package name, depending on the context. String
    exec:   function pointer
    args:   arguments of the function pointer. Tuple
    mType:  message/service/action type depending on the context
    kwargs: Keyword arguments. Dict
    ind:    thread index within the list (0..*_thread-1)
    """
    def setComponentProperties(self, which, ftype, ns='', exec=None, args=(), mType=None, \
                                kwargs={}, ind=0):
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
        elif ftype == Ftype.ACTION_SRV:
            this.kwargs = {'topic': ns, 'actType': mType, 'cb': exec}
            this.kwargs = {**this.kwargs, **kwargs}
        elif ftype == Ftype.ACTION_CLI:
            this.kwargs = {'topic': ns, 'actType': mType, 'feedback_cb': exec}
            this.kwargs = {**this.kwargs, **kwargs}
        else:
            print("ERROR: type not implemented.")

    """Helper function to turn on the module from within"""
    def turnOn(self):
        if isinstance(self.managerHandle, Manager):
            self.managerHandle.turnOn(self)
        else:
            print("ERROR: this module is not registered to a manager.")

    """Helper function to turn off the module from within"""
    def turnOff(self):
        if isinstance(self.managerHandle, Manager):
            self.managerHandle.turnOff(self)
        else:
            print("ERROR: this module is not registered to a manager.")

    """Helper function to return EX thread handle"""
    def getEXhandle(self):
        if isinstance(self.managerHandle, Manager):
            l, res = self.managerHandle.moduleIsOn(self)
            if l==1:
                return res[0][1]
            elif l>1:
                print("ERROR: Duplicate module with name", self.name, ".")
        return None

    """Helper function to return EV thread handle"""
    def getEVhandle(self):
        if isinstance(self.managerHandle, Manager):
            l, res = self.managerHandle.moduleIsOn(self)
            if l==1:
                return res[0][2]
            elif l>1:
                print("ERROR: Duplicate module with name", self.name, ".")
        return None

    """Helper function to return ES thread handle"""
    def getEShandle(self):
        if isinstance(self.managerHandle, Manager):
            l, res = self.managerHandle.moduleIsOff(self)
            if l==1:
                return res[0][1]
            elif l>1:
                print("ERROR: Duplicate module with name", self.name, ".")
        return None

"""
Base class of a module manager and decider.
Manages a given set of modules, documenting their states being either off, ready (preempted), or on.
Turn on or off the modules based on request, or through self adaptation using the basicDecider.
The basicDecider turns off a module as either its performance or resource utilization exceeds 1.
The basicDecider then search within its managed module to start the one with minimum utilization,
taking the larger one of the two aspects.
"""
class Manager:
    def __init__(self, launcher, managedModules=[], name="", freq=5.0, minReconfigInterval = 1.0):
        self.name = name
        self.launcher = None
        if isinstance(launcher, Launcher):
            self.launcher = launcher
        else:
            raise TypeError("Not initializing the manager with the correct ROS launcher.")
        rospy.on_shutdown(self.shutdown)
        # No memory copy
        self.moduleDict = dict.fromkeys(item for item in managedModules)   # make it a dict of class pointers
        self.lock = threading.Lock()
        self.onDict = dict()      # Dictionary of modules turned ON.    key = module, values = ((ex), (ev))
        self.readyDict = dict()   # Dictionary of preempted modules.    key = module, values = None
        self.offDict = dict()     # Dictionary of modules turned OFF.   key = module, values = (es)
        self.Decider = ModuleComponent(Ftype.TIMER, (freq, self.basicDecider), {})
        self.Decider_proc = None
        self.DeciderCooldownDuration = minReconfigInterval
        for m in managedModules:
            if isinstance(m, ReFrESH_Module):
                # register handler
                m.managerHandle = self
                # turn on ES
                es = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                            for th in m.ES)
                self.offDict.update({m: es})
                print("INFO: Module", m.name, "SPAWNED with Manager", self.name, ".")
            else:
                raise TypeError("Not initializing Manager", self.name, \
                                "with the supported module class: ReFrESH_Module.")

    """ Determine if a given module is ON. This function is of O(1) complexity """
    def moduleIsOn(self, module):
        return module in self.onDict

    """ Determine if a given module is OFF. Has O(1) complexity """
    def moduleIsOff(self, module):
        return module in self.offDict

    """
    If this is a preemptive module, when it turns on, it will preempt all other modules with
    prio<=this.prio, unless preempted by a module with prio>this.prio (return true).
    If this is a preemptive module and is already on, it will be preempted by another preemptive
    module with prio >= this.prio (return true).
    If this is a non-preemptive module, it should always give way to preemptive modules with
    prio >= this.prio (return true).
    This function determines if a module should be preempted by other modules. Complexity is O(n)
    """
    def moduleIsPreemptible(self, module):
        if module.preemptive and self.moduleIsOff(module):
            res = tuple(m for m in self.onDict if (m.priority > module.priority and m.preemptive))
        else:
            res = tuple(m for m in self.onDict if (m.priority >= module.priority and m.preemptive))
        return len(res), res
    
    """ Determine if a module can preempt other modules. Complexity O(n) """
    def moduleCanPreempt(self, module):
        if module.preemptive:
            if self.moduleIsOff(module):
                res = tuple(m for m in self.onDict if m.priority <= module.priority)
            else:
                res = tuple(m for m in self.onDict if m.priority < module.priority)
        else:
            res = ()
        return len(res), res

    """ Turn ON a module that was in OFF state. Preemption of other modules is considered. """
    def turnOn(self, module):
        if module in self.moduleDict:
            self.lock.acquire()
            isOn = self.moduleIsOn(module)
            if isOn:
                # module is already on
                self.lock.release()
                print("ERROR: ON request for module", module.name, ", which is already ON.")
            else:
                isPe, res_pe = self.moduleIsPreemptible(module)
                if isPe:
                    # a preemptive task with higher priority is running. add to ready set instead.
                    self.readyDict.update({module: None})
                    self.lock.release()
                    print("INFO: Module", res_pe[0].name, "preempted", module.name, ".")
                    return
                # we are safe to start this module.
                # turn off ES
                isOff = self.moduleIsOff(module)
                if isOff:
                    esHandles = self.offDict.pop(module)
                    for th in esHandles:
                        self.launcher.stop(th)
                else:
                    self.lock.release()
                    raise RuntimeError("The managed module,", module.name, \
                                        ", is neither ON nor OFF. Something bad happened.")
                # turn on EX
                ex = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                            for th in module.EX)
                # turn on EV
                ev = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                            for th in module.EV)
                # add (module, proc) to self.onDict
                self.onDict.update({module : (ex, ev)})
                print("INFO: Module", module.name, "ON.")
                # if preemptive, this module preempts all other modules with lower priority
                isPe, peTup = self.moduleCanPreempt(module)
                if isPe:
                    for m in peTup:
                        # remove m from self.onDict
                        exHandle, evHandle = self.onDict.pop(m)
                        #self.turnOff(m)
                        # turn off EV
                        for th in evHandle:
                            self.launcher.stop(th)
                        # turn off EX
                        for th in exHandle:
                            self.launcher.stop(th)
                        # turn on ES
                        es = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                                    for th in m.ES)
                        self.offDict.update({m: es})
                        self.readyDict.update({m: None})
                        print("INFO: Module", module.name, "preempted", m.name, ".")
                self.lock.release()
        else:
            print("ERROR: Module", module, "(name:", module.name, \
                    ") is not managed by this instance.")

    """ Turn OFF a module from ON state. Recovery of other modules from preemption is considered. """
    def turnOff(self, module):
        if module in self.moduleDict:
            self.lock.acquire()
            isOn = self.moduleIsOn(module)
            if isOn:
                # remove module from self.onSet
                exHandle, evHandle = self.onDict.pop(module)
                # turn off EV
                for th in evHandle:
                    self.launcher.stop(th)
                # turn off EX
                for th in exHandle:
                    self.launcher.stop(th)
                isOff = self.moduleIsOff(module)
                if not isOff:
                    # turn on ES
                    es = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                                for th in module.ES)
                    self.offDict.update({module: es})
                else:
                    self.lock.release()
                    raise RuntimeError("The managed module,", module.name, \
                                        ", is both ON and OFF. Something bad happened.")
                print("INFO: Module", module.name, "OFF.")
                # we just turned off a preemptive module... Is there anything previously preempted?
                if module.preemptive:
                    # O(n^2) complexity in the worst case.
                    while len(self.readyDict):
                        prio = - sys.maxsize - 1
                        nextOn = None
                        for m in self.readyDict:
                            if m.priority > prio:
                                prio = m.priority
                                nextOn = m
                        if nextOn:
                            # turn on m
                            #self.turnOn(nextOn)
                            # turn off ES
                            isOff = self.moduleIsOff(nextOn)
                            if isOff:
                                esHandles = self.offDict.pop(nextOn)
                                for th in esHandles:
                                    self.launcher.stop(th)
                                # turn on EX
                                ex = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                                            for th in nextOn.EX)
                                # turn on EV
                                ev = tuple(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)) \
                                            for th in nextOn.EV)
                                # add (module, proc) to self.onSet
                                self.onDict.update({nextOn: (ex, ev)})
                            self.readyDict.pop(nextOn)
                            print("INFO: Module", nextOn.name, "ON from preemption.")
                            # we hit another preemptive task with lower priority, but still the highest priority
                            # among previously suspended tasks. So we stop here.
                            if nextOn.preemptive:
                                break
                self.lock.release()
            else:
                if module in self.readyDict:
                    self.readyDict.pop(module)
                    self.lock.release()
                    print("INFO: Module", module.name, "OFF after preemption.")
                else:
                    self.lock.release()
                    print("ERROR: OFF request for module", module.name, ", which is already OFF.")
        else:
            print("ERROR: Module", module, "(name:", module.name, \
                    ") is not managed by this instance.")

    """
    Implements the Basic functionality of a decider: 
    Go over all turned ON modules and look for performannce bottlenecks that falls bellow bounds (>1.0)
    If any found, search in the list of OFF modules for alternatives.
    If alternative is found, turn off the active module and enable the alternative. and sleep for the minimum interval.
    Assume the EV and ES use a shared object of reconfig metric.
    """
    def basicDecider(self, event):
        toOn = None
        toOff = None
        self.lock.acquire()
        #O(n^2) complexity, in the worst case.
        for module in self.onDict:
            bottleNeck = module.reconfigMetric.bottleNeck()
            if bottleNeck >= 1.:
                # performance crisis. do reconfig
                print("INFO: Module", module.name, "falls below (", bottleNeck, "x) desired performance bounds.")
                candidate = module
                bestPerf = 1.
                for m in self.offDict:
                    if m != module:
                        tmp = m.reconfigMetric.bottleNeck()
                        if tmp < bestPerf:
                            candidate = m
                            bestPerf = tmp
                    else:
                        raise RuntimeError("The managed module,", module.name, \
                                        ", is both ON and OFF. Something bad happened.")
                if candidate != module:
                    print("INFO: Found alternative module", candidate.name, \
                            "with estimated performance", \
                            candidate.reconfigMetric.bottleNeck(), ".")
                    toOff = module
                    toOn = candidate
                    # we have found a solution.
                    break
                else:
                    print("WARN: No alternative module satisfies resource and performance bounds. Current value:", \
                        module.reconfigMetric.bottleNeck(), ".")
        self.lock.release()
        if toOn and toOff:
            self.turnOff(toOff)
            # you may have a bunch of things resumed after a preemptive module is off, so double check here.
            isOn = self.moduleIsOn(toOn)
            if not isOn:
                self.turnOn(toOn)
            time.sleep(self.DeciderCooldownDuration)

    """ Turn on a list of modules from external request """
    def requestOn(self, onList):
        # hash into Dictionary for O(1) complexity
        onDictL = dict.fromkeys(onList)
        toOn = set()
        # O(n) complexity
        for module in self.offDict:
            if module.name in onDictL:
                toOn.add(module)
            elif module in onDictL:
                toOn.add(module)
        for module in toOn:
            self.turnOn(module)

    """ Turn off a list of modules from external request """
    def requestOff(self, offList):
        offDictL = dict.fromkeys(offList)
        toOff = set()
        for module in self.onDict:
            if module.name in offDictL:
                toOff.add(module)
            elif module in offDictL:
                toOff.add(module)
        for module in self.readyDict:
            if module.name in offDictL:
                toOff.add(module)
            elif module in offDictL:
                toOff.add(module)
        for module in toOff:
            self.turnOff(module)

    """ Starts decider object """
    def run(self, blocking = False):
        self.Decider_proc = self.launcher.launch(self.Decider.ftype, \
                                                *tuple(self.Decider.args), \
                                                **dict(self.Decider.kwargs))
        print("INFO: Launched decider thread.")
        if blocking:
            self.launcher.spin()

    """ Cleanup """
    def shutdown(self):
        self.launcher.stop(self.Decider_proc)
        self.lock.acquire()
        while len(self.onDict):
            m, (exHandle, evHandle) = self.onDict.popitem()
            # turn off EV
            for th in evHandle:
                self.launcher.stop(th)
            # turn off EX
            for th in exHandle:
                self.launcher.stop(th)
            print("INFO: Module", m.name, "SHUTDOWN.")
        while len(self.offDict):
            m, esHandle = self.offDict.popitem()
            # turn off ES
            for th in esHandle:
                self.launcher.stop(th)
            print("INFO: Module", m.name, "SHUTDOWN.")
        self.readyDict.clear()
        self.moduleDict.clear()
        self.lock.release()

if __name__ == "__main__":
    # a simple test case with two empty modules.
    taskLauncher = Launcher("RobotTaskRunner")
    testModule1 = ReFrESH_Module("test1", priority=10, preemptive=True)
    testModule2 = ReFrESH_Module("test2")
    taskManager = Manager(taskLauncher, [testModule1, testModule2])
    # non-blocking run
    taskManager.run()
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
    taskLauncher.spin()
    # shutdown
    #taskManager.shutdown()
    taskLauncher.shutdown()
