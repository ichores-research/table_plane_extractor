#!/usr/bin/python3
from math import atan, pi
from table_plane_extractor.srv import TablePlaneExtractor, TablePlaneExtractorResponse
from table_plane_extractor.msg import Plane
import numpy as np
import rospy
import open3d as o3d
import compas.geometry.bbox as compas_bb
from open3d_ros_helper import open3d_ros_helper as orh
import tf2_ros
from v4r_util.util import o3d_bb_to_ros_bb, transformPointCloud, get_minimum_oriented_bounding_box
from vision_msgs.msg import BoundingBox3DArray


def table_plane_extractor_methode(req):
    ''' 
    Table plane extractor who takes the point cloud as input. Returns possible horizontal planes 
    with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and bounding boxes around the planes.
    Input: sensor_msgs/PointCloud2 inputCloud
    Output: table_plane_extractor/Plane[] planes, vision_msgs/BoundingBox3DArray plane_bounding_boxes
    '''
    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    #tf_listener = tf.TransformListener()
    # tf buffer for tf transform
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
    pcd = req.point_cloud
    pcd = transformPointCloud(pcd, base_frame, pcd.header.frame_id, tf_buffer)
    header = pcd.header
    pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

    # downsample cloud
    downsample_vox_size = rospy.get_param(
        "/table_plane_extractor/downsample_vox_size")
    pcd = pcd.voxel_down_sample(voxel_size=downsample_vox_size)

    planes = []
    bb_arr = BoundingBox3DArray()
    bb_arr.boxes = []
    bb_arr.header = header

    cluster_dbscan_eps = rospy.get_param(
        "/table_plane_extractor/cluster_dbscan_eps")
    min_cluster_size = rospy.get_param(
        "/table_plane_extractor/min_cluster_size")
    max_angle_deg = rospy.get_param("/table_plane_extractor/max_angle_deg")
    z_min = rospy.get_param("/table_plane_extractor/z_min")
    distance_threshold = rospy.get_param(
        "/table_plane_extractor/plane_segmentation_distance_threshold")
    max_angle_rad = max_angle_deg * pi/180

    # filter all points below z_min
    points = np.array(pcd.points)
    floor = points[:, 2] < z_min
    pcd.points = o3d.utility.Vector3dVector(points[floor == 0])

    # get planes with RANSAC
    while len(pcd.points) > min_cluster_size:
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        # ax + by + cz + d = 0
        [a, b, c, d] = plane_model

        # remove non-horizontal-planes
        # z = (-ax - by - d)/c -> gradient = (-a/c, -b/c)
        if (abs(atan(-a/c)) > max_angle_rad or abs(atan(-b/c)) > max_angle_rad):
            pcd = pcd.select_by_index(inliers, invert=True)
            continue

        inlier_cloud = pcd.select_by_index(inliers)
        pcd = pcd.select_by_index(inliers, invert=True)
        # segment plane-pointcloud because multiple objects could potentially align with the same plane
        idx = inlier_cloud.cluster_dbscan(cluster_dbscan_eps, min_cluster_size)
        values = np.unique(idx)
        for val in values:
            if val == -1:
                continue
            # select clustered plane cloud
            cluster_idx = np.where(np.asarray(idx) == val)[0]
            plane_pc = inlier_cloud.select_by_index(cluster_idx)
            bb_plane = plane_pc.get_oriented_bounding_box()
            planes.append(Plane(a, b, c, d))
            print("Plane equation: {}x + {}y + {}z + {} = 0".format(a, b, c, d))
            bb_plane = get_minimum_oriented_bounding_box(plane_pc)
            bb_arr.boxes.append(o3d_bb_to_ros_bb(bb_plane))

    return TablePlaneExtractorResponse(planes, bb_arr)


def table_plane_extractor_server():
    ''' 
    Starting table plane extractor server node
    Topic: /test/table_plane_extractor
    '''
    rospy.init_node('table_plane_extractor_server')
    s = rospy.Service('/test/table_plane_extractor',
                      TablePlaneExtractor, table_plane_extractor_methode)
    print("Ready to extract planes")
    rospy.spin()


if __name__ == "__main__":
    table_plane_extractor_server()
