#!/usr/bin/python

from distutils.log import error
import roslaunch
import rospy
import actionlib
from actionlib.action_server import nop_cb
from enum import Enum
import threading
import inspect
import ctypes

def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    if not inspect.isclass(exctype):
        raise TypeError("Only types can be raised (not instances)")
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble, 
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, 0)
        raise SystemError("PyThreadState_SetAsyncExc failed")

class Thread(threading.Thread):
    def _get_my_tid(self):
        """determines this (self's) thread id"""
        if not self.isAlive():
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
        self.raise_exc(SystemExit)
        self.join()

class Ftype(Enum):
    NODE = 0
    LAUNCH_FILE = 1
    THREAD = 2
    TIMER = 3
    SUBSCRIBER = 4
    SERVICE = 5
    ACTION = 6

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
    
    def __del__(self):
        self.nodeLauncher.stop()
        self.roscoreProc.shutdown()

class ModuleComponent:
    def __init__(self, ftype, args, kwargs):
        self.ftype = ftype
        self.args = args
        self.kwargs = kwargs

class ReFrESH_Module:
    def __init__(self, name):
        self.name = name
        self.EX = ModuleComponent(Ftype.THREAD, (), {})
        self.EV = ModuleComponent(Ftype.THREAD, (), {})
        self.ES = ModuleComponent(Ftype.THREAD, (), {})
    
    def setComponentProperties(self, which, ftype, ns=None, exec=None, args=None, mType=None, kwargs=None):
        this = None
        if (which in ['EX', 'ex', 'Execute', 'execute', 'Executor', 'executer', self.EX]):
            this = self.EX
        elif (which in ['EV', 'ev', 'Evaluate', 'evaluate', 'Evaluator', 'evaluator', self.EV]):
            this = self.EV
        elif (which in ['ES', 'es', 'Estimate', 'estimate', 'Estimator', 'estimator', self.ES]):
            this = self.ES
        else:
            error("ERROR: Invalid set-property request!")
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

class Manager:
    def __init__(self, launcher, managedModules=[]):
        self.launcher = None
        if isinstance(launcher, Launcher):
            self.launcher = launcher
        else:
            error("ERROR: not initializing the manager with the correct ROS launcher.")
        # No memory copy
        self.moduleSet = set(item.name for item in managedModules) # make it a name set
        self.onSet = set()
        self.offSet = set()
        self.Decider = ModuleComponent(Ftype.THREAD, (), {})
        self.Decider_proc = None
        for m in managedModules:
            if isinstance(m, ReFrESH_Module):
                # turn on ES
                es = self.launcher.launch(m.ES.ftype, *tuple(m.ES.args), **dict(m.ES.kwargs))
                self.offSet.add((m.name, es))
            else:
                error("ERROR: not initializing the manager with the supported module class: ReFrESH_Module.")
        
    def turnOn(self, module):
        if module.name in self.moduleSet:
            res = [m for m in self.onSet if m[0]==module.name]
            if len(res):
                # module is already on
                print("ERROR: ON request for module "+res[0]+", which is already ON.")
            else:
                # turn off ES
                res_off = [m_off for m_off in self.offSet if m_off[0]==module.name]
                if len(res_off):
                    for item in res_off:
                        self.offSet.remove(item)
                        self.launcher.stop(item[1])
                else:
                    error("ERROR: The managed module is neither on nor off. Something bad happened.")
                # turn on EX
                ex = self.launcher.launch(module.EX.ftype, *tuple(module.EX.args), **dict(module.EX.kwargs))
                # turn on EV
                ev = self.launcher.launch(module.EV.ftype, *tuple(module.EV.args), **dict(module.EV.kwargs))
                # add (module, proc) to self.onSet
                self.onSet.add((module.name, ex, ev))
        else:
            print("ERROR: Module "+str(res[0])+" is not managed by this instance.")
    
    def turnOff(self, module):
        if module.name in self.moduleSet:
            res = [m for m in self.onSet if m[0]==module.name]
            if len(res):
                for item in res:
                    # remove res from self.onSet
                    self.onSet.remove(item)
                    # turn off EX
                    self.launcher.stop(item[1])
                    # turn off EV
                    self.launcher.stop(item[2])
                res_off = [m_off for m_off in self.offSet if m_off[0]==module.name]
                if len(res_off):
                    error("ERROR: The managed module is both on and off. Something bad happened.")
                else:
                    # turn on ES
                    es = self.launcher.launch(module.ES.ftype, *tuple(module.ES.args), **dict(module.ES.kwargs))
                    self.offSet.add((module.name, es))
            else:
                print("ERROR: OFF request for module "+str(res[0])+", which is already OFF.")
        else:
            print("ERROR: Module "+str(res[0])+" is not managed by this instance.")
    
    def run(self):
        self.Decider_proc = self.launcher.launch(self.Decider.ftype, *tuple(self.Decider.args), **dict(self.Decider.kwargs))
        print("INFO: Launched decider thread.")
        rospy.spin()
    
    def __del__(self):
        for m in self.onSet:
            # turn off EX
            self.launcher.stop(m[1])
            # turn off EV
            self.launcher.stop(m[2])
        for m in self.moduleSet:
            # turn off ES
            self.launcher.stop(m[1])

if __name__ == "__main__":
    taskLauncher = Launcher("RobotTaskRunner")
    taskManager = Manager(taskLauncher, [])
    taskManager.run()
    del taskManager