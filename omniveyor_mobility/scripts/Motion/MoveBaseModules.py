#!/usr/bin/python

from PlannerModules import MoveBaseModule
import dynamic_reconfigure.client as drcli

""" Using TEB local planner to avoid local obstacles, potentially dynamic """
class tebLocalPlannerROSModule(MoveBaseModule):
    def __init__(self, name:str="tebLocalPlannerROSMotion", priority:int=96, preemptive:bool=True):
        super().__init__(name, priority, preemptive,
                        bgp="navfn/NavfnROS", blp="teb_local_planner/TebLocalPlannerROS")
        self.defaultPerformanceMetrics = [0.0, 0.0]
        self.performanceMetrics = [0.0, 0.0]

    def updatePlannerPrecisionTolerance(self):
        super().updatePlannerPrecisionTolerance()
        linTol = self.currentActionGoal.goal_tolerance[0][0]
        angTol = self.currentActionGoal.goal_tolerance[0][3]
        cfg = {'xy_goal_tolerance': linTol, 'yaw_goal_tolerance': angTol}
        try:
            drc = drcli.Client("move_base/TebLocalPlannerROS", timeout=10.0, config_callback=None)
            #print(drc.get_configuration())
            drc.update_configuration(cfg)
            drc.close()
        except Exception as e:
            print('ERROR: Dynamic Reconfigure Client of', self.name, 'FAILED:', e)

""" Using DWA local planner to refine the global trajectory for obstacle avoidance """
class dwaPlannerROSModule(MoveBaseModule):
    def __init__(self, name:str="dwaPlannerROSMotion", priority:int=95, preemptive:bool=True):
        super().__init__(name, priority, preemptive,
                        bgp="navfn/NavfnROS", blp="dwa_local_planner/DWAPlannerROS")
        self.defaultPerformanceMetrics = [0.3, 0.3]
        self.performanceMetrics = [0.3, 0.3]

    def updatePlannerPrecisionTolerance(self):
        super().updatePlannerPrecisionTolerance()
        linTol = self.currentActionGoal.goal_tolerance[0][0]
        angTol = self.currentActionGoal.goal_tolerance[0][3]
        cfg = {'xy_goal_tolerance': linTol, 'yaw_goal_tolerance': angTol}
        try:
            drc = drcli.Client("move_base/DWAPlannerROS", timeout=10.0, config_callback=None)
            drc.update_configuration(cfg)
            drc.close()
        except Exception as e:
            print('ERROR: Dynamic Reconfigure Client of', self.name, 'FAILED:', e)

""" Follows a global plan as-is with PID controller"""
class pidControllerModule(MoveBaseModule):
    def __init__(self, name:str="pidControllerMotion", priority:int=94, preemptive:bool=True):
        super().__init__(name, priority, preemptive,
                        bgp="navfn/NavfnROS", blp="pid_controller/PIDController")
        self.defaultPerformanceMetrics = [0.5, 0.5]
        self.performanceMetrics = [0.5, 0.5]

    def updatePlannerPrecisionTolerance(self):
        super().updatePlannerPrecisionTolerance()
        linTol = self.currentActionGoal.goal_tolerance[0][0]
        angTol = self.currentActionGoal.goal_tolerance[0][3]
        cfg = {'linear_precision': linTol, 'angular_precision': angTol}
        try:
            drc = drcli.Client("move_base/PIDController", timeout=10.0, config_callback=None)
            drc.update_configuration(cfg)
            drc.close()
        except Exception as e:
            print('ERROR: Dynamic Reconfigure Client of', self.name, 'FAILED:', e)

def test(taskManager):
    import time
    import rospy
    from geometry_msgs.msg import PoseStamped
    try:
        time.sleep(20)
        # turn on by requesting the manager
        tgt = PoseStamped()
        tgt.header.stamp = rospy.Time.now()
        tgt.pose.position.x = 1.0
        tgt.pose.orientation.w = 1.0
        taskManager.runGoal(tgt, timeout=60.0)
        tgt.header.stamp = rospy.Time.now()
        tgt.pose.position.x = 4.0
        tgt.pose.position.y = -1.0
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
    
    #from viztracer import VizTracer
    #tracer = VizTracer()
    #tracer.start()
    
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
    taskManager = MotionManager(taskLauncher, managedModules=[mbMan, kbMod]) #rmMod, jsMod
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
    
    #tracer.stop()
    #tracer.save()
