#!/usr/bin/python

from PlannerModules import MoveBaseModule

""" Using TEB local planner to avoid local obstacles, potentially dynamic """
class tebLocalPlannerROSModule(MoveBaseModule):
    def __init__(self, name="tebLocalPlannerROSMotion", priority=96, preemptive=True):
        super().__init__(name, priority, preemptive,
                        bgp="navfn/NavfnROS", blp="teb_local_planner/TebLocalPlannerROS")

""" Using DWA local planner to refine the global trajectory for obstacle avoidance """
class dwaPlannerROSModule(MoveBaseModule):
    def __init__(self, name="dwaPlannerROSMotion", priority=95, preemptive=True):
        super().__init__(name, priority, preemptive,
                        bgp="navfn/NavfnROS", blp="dwa_local_planner/DWAPlannerROS")

""" Follows a global plan as-is with PID controller"""
class pidControllerModule(MoveBaseModule):
    def __init__(self, name="pidControllerMotion", priority=94, preemptive=True):
        super().__init__(name, priority, preemptive,
                        bgp="navfn/NavfnROS", blp="pid_controller/PIDController")

def test(taskManager):
    import time
    from geometry_msgs.msg import PoseStamped
    try:
        time.sleep(20)
        # turn on by requesting the manager
        tgt = PoseStamped()
        tgt.pose.position.x = -1.0
        tgt.pose.orientation.w = 1.0
        taskManager.runGoal(tgt)
        tgt.pose.position.x = 1.0
        tgt.pose.position.y = -2.0
        tgt.pose.orientation.w = 1.0
        taskManager.runGoal(tgt)
    except SystemExit:
        return

if __name__ == "__main__":
    import os
    import sys
    currentdir = os.path.dirname(os.path.realpath(__file__))
    parentdir = os.path.dirname(currentdir)
    sys.path.append(parentdir)
    from ReFrESH_ROS_utils import Launcher, Thread, Ftype
    from Managers import MotionManager, MoveBaseManager
    from TeleopModules import joystickTeleopModule, remoteTeleopModule, keyboardTeleopModule
    # a simple test case with three teleop modules.
    taskLauncher = Launcher("motionManager")
    simThread = taskLauncher.launch(Ftype.LAUNCH_FILE, pkgName='omniveyor_gazebo_world', fileName='IMI.launch')
    mappingThread = taskLauncher.launch(Ftype.LAUNCH_FILE, pkgName='omniveyor_mobility', fileName='map_and_localization.launch', args=('static_map:=0',))
    jsMod = joystickTeleopModule()
    rmMod = remoteTeleopModule()
    kbMod = keyboardTeleopModule()
    teb = tebLocalPlannerROSModule()
    dwa = dwaPlannerROSModule()
    pid = pidControllerModule()
    mbMan = MoveBaseManager(taskLauncher, managedModules=[teb, dwa, pid])
    taskManager = MotionManager(taskLauncher, managedModules=[mbMan])#, jsMod, rmMod, kbMod])
    t = Thread(target=test, args=(taskManager,))
    t.start()
    # blocking run
    taskManager.run(blocking = True)
    # shutdown
    t.stop()
    taskLauncher.stop(simThread)
    taskLauncher.stop(mappingThread)
    taskManager.shutdown()
    taskLauncher.shutdown() 
