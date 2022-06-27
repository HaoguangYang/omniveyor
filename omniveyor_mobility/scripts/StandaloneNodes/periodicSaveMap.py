#!/usr/bin/python

from slam_toolbox_msgs.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String
from spatio_temporal_voxel_layer.srv import SaveGrid
import time
import rospy
import rospkg

if __name__ == "__main__":
    rospy.init_node("map_saving_utility")
    mapPath = rospy.get_param("~map_save_path", rospkg.RosPack().get_path('omniveyor_mobility')+'/resources/maps/map')
    interval = rospy.get_param("~map_save_interval", 60.0)
    if (interval < 0.):
        raise RuntimeError("ERROR: map saving interval must not be a negative number!")
    planarMapSaveSrvUp = False
    pointCloudSaveSrvUp = False
    save2dMap = None
    save3dMap = None

    f = rospy.Rate(1.0)
    startupTime = time.time() + interval

    while(not rospy.is_shutdown()):

        f.sleep()

        if (time.time() <= startupTime):
            continue
        startupTime = time.time() + interval
        
        if (not planarMapSaveSrvUp):
            try:
                rospy.wait_for_service("slam_toolbox/serialize_map", timeout=1.0)
                planarMapSaveSrvUp = True
                save2dMap = [rospy.ServiceProxy('slam_toolbox/serialize_map', SerializePoseGraph),
                            rospy.ServiceProxy('slam_toolbox/save_map', SaveMap)]
            except rospy.ROSException as ex:
                print ("WARNING: Service 'slam_toolbox/serialize_map' or 'slam_toolbox/save_map' is down")
    
        if (not pointCloudSaveSrvUp):
            try:
                rospy.wait_for_service("/move_base/global_costmap/rgbd_obstacle_layer_d1/spatiotemporal_voxel_grid/save_grid", timeout=1.0)
                rospy.wait_for_service("/move_base/global_costmap/rgbd_obstacle_layer_d2/spatiotemporal_voxel_grid/save_grid", timeout=1.0)
                pointCloudSaveSrvUp = True
                save3dMap = [rospy.ServiceProxy('/move_base/global_costmap/rgbd_obstacle_layer_d1/spatiotemporal_voxel_grid/save_grid', SaveGrid),
                             rospy.ServiceProxy('/move_base/global_costmap/rgbd_obstacle_layer_d2/spatiotemporal_voxel_grid/save_grid', SaveGrid)]
            except rospy.ROSException as ex:
                print ("WARNING: Service 'spatiotemporal_voxel_grid/save_grid' is down")

        if (planarMapSaveSrvUp):
            try:
                resp = save2dMap[0](mapPath)
                print("PoseGraph saved with response: " + str(resp))
            except rospy.ServiceException as exc:
                print("WARNING: Service 'slam_toolbox/serialize_map' did not process request: " + str(exc))
            try:
                resp = save2dMap[1](String(mapPath))
                print("pgm saved with response: " + str(resp))
            except rospy.ServiceException as exc:
                print("WARNING: Service 'slam_toolbox/save_map' did not process request: " + str(exc))
        
        if (pointCloudSaveSrvUp):
            try:
                resp = save3dMap[0](String(mapPath+"_1.vdb"))
                print("vdb (front) saved with response: " + str(resp))
                resp = save3dMap[1](String(mapPath+"_2.vdb"))
                print("vdb (rear) saved with response: " + str(resp))
            except rospy.ServiceException as exc:
                print("WARNING: Service 'spatiotemporal_voxel_grid/save_grid' did not process request: " + str(exc))
