#!/usr/bin/python3
from math import atan, pi
from table_plane_extractor.srv import TablePlaneExtractor, TablePlaneExtractorResponse
from table_plane_extractor.msg import Plane
from extractor_of_table_planes import extract_table_planes_from_pcd
import numpy as np
import rospy
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import tf2_ros
from v4r_util.util import o3d_bb_to_ros_bb, transformPointCloud, get_minimum_oriented_bounding_box, ros_bb_arr_to_rviz_marker_arr
from vision_msgs.msg import BoundingBox3DArray
from visualization_msgs.msg import MarkerArray

from std_msgs.msg import Header



def table_plane_extractor_methode(req):
    ''' 
    Table plane extractor who takes the point cloud as input. Returns possible horizontal planes 
    with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and bounding boxes around the planes.
    Input: sensor_msgs/PointCloud2 inputCloud
    Output: table_plane_extractor/Plane[] planes, vision_msgs/BoundingBox3DArray plane_bounding_boxes
    '''
    enable_rviz_visualization = rospy.get_param(
        '/table_objects_extractor/enable_rviz_visualization')
    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    if enable_rviz_visualization:
        pub = rospy.Publisher('PlaneBoundingBoxVisualizer',
                              MarkerArray, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
    pcd = req.point_cloud
    pcd = transformPointCloud(pcd, base_frame, pcd.header.frame_id, tf_buffer) #make sure pointcloud has z pointing up
    header = pcd.header
    pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

    # downsample cloud
    downsample_vox_size = rospy.get_param(
        "/table_plane_extractor/downsample_vox_size")
    pcd = pcd.voxel_down_sample(voxel_size=downsample_vox_size)

    bb_arr = BoundingBox3DArray()
    bb_arr.header = header

    cluster_dbscan_eps = rospy.get_param(
        "/table_plane_extractor/cluster_dbscan_eps")
    min_cluster_size = rospy.get_param(
        "/table_plane_extractor/min_cluster_size")
    max_angle_deg = rospy.get_param("/table_plane_extractor/max_angle_deg")
    z_min = rospy.get_param("/table_plane_extractor/z_min")
    distance_threshold = rospy.get_param(
        "/table_plane_extractor/plane_segmentation_distance_threshold")

    planes, bboxes = extract_table_planes_from_pcd(pcd, cluster_dbscan_eps, min_cluster_size, distance_threshold, max_angle_deg, z_min)
    
    planes_ros = [Plane(Header(0, header.stamp, base_frame), a, b, c, d) for a, b, c, d in planes]
    bb_arr.boxes = [o3d_bb_to_ros_bb(bb_plane) for bb_plane in bboxes]

    if enable_rviz_visualization:
        marker_arr = ros_bb_arr_to_rviz_marker_arr(
            bb_arr)
        pub.publish(marker_arr)
    return TablePlaneExtractorResponse(planes_ros, bb_arr)


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
