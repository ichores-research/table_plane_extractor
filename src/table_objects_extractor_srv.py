#!/usr/bin/env python3
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
import numpy as np
import tf2_ros
from visualization_msgs.msg import MarkerArray
from v4r_util.util import transformPointCloud, o3d_bb_list_to_ros_bb_arr, ros_bb_arr_to_rviz_marker_arr
from table_plane_extractor.srv import GetBBOfObjectsOnTable, GetBBOfObjectsOnTableResponse, GetPCOfObjectsOnTable, GetPCOfObjectsOnTableResponse
from extractor_of_table_planes import extract_table_planes_from_pcd
from extractor_of_table_objects import extract_objects_from_tableplane

def table_objects_extractor(ros_pcd): #TODO remove target frame also from config
    '''
    Returns bounding boxes and pointclouds of objects found on a table plane.
    Output: list[open3d.geometry.OrientedBoundingBox] bb_arr
            list[open3d.geometry.PointCloud] pc_arr
            np.array label_img (flattened image with shape (width*height))
    '''


    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
    pcd = ros_pcd
    height = pcd.height
    width = pcd.width
    pcd = transformPointCloud(pcd, base_frame, pcd.header.frame_id, tf_buffer) #make sure pointcloud has z pointing up
    header = pcd.header
    pcd_with_nans = orh.rospc_to_o3dpc(pcd, remove_nans=False)#TODO why does tableplaneextractor die with nans
    pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

    # downsample cloud
    downsample_vox_size = rospy.get_param(
        "/table_plane_extractor/downsample_vox_size")
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=downsample_vox_size)

    cluster_dbscan_eps = rospy.get_param(
        "/table_plane_extractor/cluster_dbscan_eps")
    min_cluster_size = rospy.get_param(
        "/table_plane_extractor/min_cluster_size")
    max_angle_deg = rospy.get_param("/table_plane_extractor/max_angle_deg")
    z_min = rospy.get_param("/table_plane_extractor/z_min")
    distance_threshold = rospy.get_param(
        "/table_plane_extractor/plane_segmentation_distance_threshold")

    planes, bboxes = extract_table_planes_from_pcd(
        pcd_downsampled, 
        cluster_dbscan_eps, 
        min_cluster_size, 
        distance_threshold, 
        max_angle_deg, 
        z_min)    

    eps = rospy.get_param("/table_objects_extractor/cluster_dbscan_eps")
    min_points = rospy.get_param("/table_objects_extractor/min_points")
    min_volume = rospy.get_param("/table_objects_extractor/min_volume")
    max_obj_height = rospy.get_param('/table_objects_extractor/max_obj_height')

    bb_arr, pc_arr, label_img = extract_objects_from_tableplane(
        pcd_with_nans, 
        bboxes, 
        eps, 
        min_points, 
        min_volume, 
        max_obj_height,
        height,
        width)
    return bb_arr, pc_arr, label_img


def get_object_bbs(req):
    '''
    Service that returns bounding boxes of objects found on a table plane
    Topic: /objects_on_table/get_bounding_boxes
    Input: sensor_msgs/PointCloud2 scene
    Output: vision_msgs/BoundingBox3DArray detected_objects
    '''
    enable_rviz_visualization = rospy.get_param(
        '/table_objects_extractor/enable_rviz_visualization')
    if enable_rviz_visualization:
        pub = rospy.Publisher('objectsOnTableVisualizer',
                              MarkerArray, queue_size=10)

    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    o3d_bb_arr, o3d_pc_arr, _ = table_objects_extractor(req.point_cloud)
    ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
        o3d_bb_arr, base_frame, rospy.get_rostime())

    if enable_rviz_visualization:
        marker_arr = ros_bb_arr_to_rviz_marker_arr(ros_bb_arr)
        pub.publish(marker_arr)

    return GetBBOfObjectsOnTableResponse(ros_bb_arr)


def get_object_pcs(req):
    '''Service that returns the pointclouds of objects found on a table plane
        Topic: /objects_on_table/get_point_clouds
        Input: sensor_msgs/PointCloud2 scene
        Output: sensor_msgs/PointCloud2[] detected_objects
    '''
    enable_rviz_visualization = rospy.get_param(
        '/table_objects_extractor/enable_rviz_visualization')
    if enable_rviz_visualization:
        pub = rospy.Publisher('objectsOnTableVisualizer',
                              MarkerArray, queue_size=10)

    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    o3d_bb_arr, o3d_pc_arr, _ = table_objects_extractor(req.point_cloud)
    ros_pc_arr = []
    for pc in o3d_pc_arr:
        ros_pc = orh.o3dpc_to_rospc(pc, base_frame, rospy.get_rostime())
        ros_pc_arr.append(ros_pc)

    if enable_rviz_visualization:
        ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
            o3d_bb_arr, base_frame, rospy.get_rostime())
        marker_arr = ros_bb_arr_to_rviz_marker_arr(ros_bb_arr)
        pub.publish(marker_arr)

    return GetPCOfObjectsOnTableResponse(ros_pc_arr)


def table_objects_extractor_server():
    '''
    Starts table_objects_extractor services
    Topics: /objects_on_table/get_bounding_boxes
            /objects_on_table/get_point_clouds
    '''
    rospy.init_node('table_objects_extractor')
    s_bb = rospy.Service('/objects_on_table/get_bounding_boxes',
                         GetBBOfObjectsOnTable, get_object_bbs)
    s_pc = rospy.Service('/objects_on_table/get_point_clouds',
                         GetPCOfObjectsOnTable, get_object_pcs)
    print("Ready to detect objects on table plane.")
    rospy.spin()


if __name__ == "__main__":
    try:
        table_objects_extractor_server()
    except rospy.ROSInterruptException:
        pass
