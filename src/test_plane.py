#!/usr/bin/python3

from table_plane_extractor.srv import TablePlaneExtractor
from table_plane_extractor.msg import Plane
import numpy as np
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
from sensor_msgs.msg import PointCloud2
import copy

class UseTablePlaneExtractor():
    ''' Example of using the table plane extractor service. '''
    def __init__(self):
        self.cloud = None
        # get point cloud
        topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        sub = rospy.Subscriber(topic, PointCloud2, self.pc_cb)
        rospy.wait_for_message(topic, PointCloud2)
        # wait until service is available
        rospy.wait_for_service('/test/table_plane_extractor')
        try:
            # use service
            table_extractor = rospy.ServiceProxy('/test/table_plane_extractor', TablePlaneExtractor)
            response = table_extractor(self.cloud)
            for pcd in response.clouds:
                cloud = orh.rospc_to_o3dpc(pcd)
                bb_cloud = cloud.get_oriented_bounding_box()

                # remove floor
                if bb_cloud.center[2] < 0.2:
                    continue

                # colorize the result
                bb_cloud.color = (0, 1, 0)
                bb_cloud_mod = copy.deepcopy(bb_cloud)
                bb_cloud_mod.color = (0, 0, 1)

                bb_cloud_mod.center = (bb_cloud_mod.center[0], bb_cloud_mod.center[1], (bb_cloud_mod.center[2] + (bb_cloud_mod.extent[2] / 2)) / 2)
                # extent result for better collision avoidance
                bb_cloud_mod.extent = (bb_cloud_mod.extent[0]+0.04, bb_cloud_mod.extent[1]+0.04, bb_cloud.center[2] + (bb_cloud_mod.extent[2] / 2))

                # visualize result
                o3d.visualization.draw_geometries([cloud, bb_cloud, bb_cloud_mod])

        except rospy.ServiceException as e:
            print(e)

    def pc_cb(self, data):
        self.cloud = data

if __name__ == "__main__":
    rospy.init_node('table_plane_extractor_client')
    u = UseTablePlaneExtractor()