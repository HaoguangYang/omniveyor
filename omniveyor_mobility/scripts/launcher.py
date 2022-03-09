#!/usr/bin/python

import roslaunch
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
    
    def shutdown(self):
        """raises SystemExit in the context of the given thread, which should 
        cause the thread to exit silently (unless caught)"""
        self.raise_exc(SystemExit)
        self.join()

class Launcher:
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.roscoreProc = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], is_core=True)
        self.roscoreProc.start()
        self.nodeLauncher = roslaunch.scriptapi.ROSLaunch()
        self.nodeLauncher.start()
    
    def nodeLaunch(self, pkgName, execName, name=None, namespace='/', args='', respawn=False, \
                 respawn_delay=0.0, remap_args=None, env_args=None, output=None, launch_prefix=None):
        try:
            nodeProc = self.nodeLauncher.launch(roslaunch.core.Node(pkgName, execName, name=name, \
                namespace=namespace, args=args, respawn=respawn, respawn_delay=respawn_delay, \
                remap_args=remap_args, env_args=env_args, output=output, launch_prefix=launch_prefix))
        except:
            nodeProc = None
        return nodeProc
    
    def fileLaunch(self, pkgName, fileName, fullPathList=[]):
        if len(fullPathList):
            launchProc = roslaunch.parent.ROSLaunchParent(self.uuid, fullPathList)
            launchProc.start()
            return launchProc
        else:
            fullPath = roslaunch.rlutil.resolve_launch_arguments([pkgName, fileName])
            launchProc = roslaunch.parent.ROSLaunchParent(self.uuid, fullPath)
            launchProc.start()
            return launchProc
    
    def threadLaunch(self, funcPtr, args):
        t = Thread(target=funcPtr, args=args)
        t.start()
        return t
    
    def stop(self, proc):
        if callable(getattr(proc, "stop", None)):
            proc.stop()
        elif callable(getattr(proc, "shutdown", None)):
            proc.shutdown()
    
    def cleanup(self):
        self.nodeLauncher.stop()
        self.roscoreProc.shutdown()

