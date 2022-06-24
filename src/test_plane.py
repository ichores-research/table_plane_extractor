#!/usr/bin/python3

from table_plane_extractor.srv import TablePlaneExtractor
from table_plane_extractor.msg import Plane
import numpy as np
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
from sensor_msgs.msg import PointCloud2
import copy


def use_table_plane_extractor():
    
    topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
    rospy.wait_for_service('/test/table_plane_extractor')

    try:
        table_extractor = rospy.ServiceProxy('/test/table_plane_extractor', TablePlaneExtractor)
        response = table_extractor(topic)
        for pcd in response.clouds:
            cloud = orh.rospc_to_o3dpc(pcd)
            bb_cloud = cloud.get_oriented_bounding_box()
            bb_cloud.color = (0, 1, 0)
            print(bb_cloud.center)
            print(bb_cloud.extent)
            
            #new_bb_cloud = copy.deepcopy(bb_cloud)
            #new_bb_cloud.center = [bb_cloud.center[0], bb_cloud.center[1], bb_cloud.center[2] ]
            o3d.visualization.draw_geometries([cloud, bb_cloud])
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    rospy.init_node('table_plane_extractor_client')
    use_table_plane_extractor()