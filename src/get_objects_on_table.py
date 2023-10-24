#!/usr/bin/env python3
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
import numpy as np
import tf2_ros
from visualization_msgs.msg import MarkerArray
from v4r_util.util import ros_bb_to_o3d_bb, transformPointCloud, o3d_bb_list_to_ros_bb_arr, ros_bb_arr_to_rviz_marker_arr, get_minimum_oriented_bounding_box
from table_plane_extractor.srv import GetBBOfObjectsOnTable, GetBBOfObjectsOnTableResponse, GetPCOfObjectsOnTable, GetPCOfObjectsOnTableResponse
from extractor_of_table_planes import extract_table_planes_from_pcd


def getter_of_objects_on_table(pcd, table_planes, eps, min_points, min_volume, max_obj_height, height, width):
    print(np.asarray(pcd.points).shape)
    print(height*width)
    label_img = np.full(height * width, -1, dtype=np.int16)
    pc_arr = []
    bb_arr = []

    for plane_bb in table_planes:

        # get bounding box above table
        bb_above_table = plane_bb
        bb_above_table.center = (bb_above_table.center[0], bb_above_table.center[1],
                                 bb_above_table.center[2]+max_obj_height/2.0+bb_above_table.extent[2])
        bb_above_table.extent = (
            bb_above_table.extent[0]+0.04, bb_above_table.extent[1]+0.04, bb_above_table.extent[2]+max_obj_height)

        # filter out points that are not above the table
        indices = np.array(
            bb_above_table.get_point_indices_within_bounding_box(pcd.points))
        scene_above_table = pcd.select_by_index(indices)
        print(np.asarray(scene_above_table.points).shape)
        # segment scene-pointcloud into objects
        labels = np.array(scene_above_table.cluster_dbscan(
            eps=eps, min_points=min_points))
        labels_unique = np.unique(labels)
        print(labels_unique)
        # get bounding box and pointcloud for each object
        for label in labels_unique:
            if label == -1:
                continue
            obj_indices = np.where(labels == label)
            obj = scene_above_table.select_by_index(obj_indices[0])
            obj_bb = get_minimum_oriented_bounding_box(obj)
            # filter very small bounding boxes
            if obj_bb.volume() < min_volume:
                labels[labels == label] = -1
                continue
            pc_arr.append(obj)
            bb_arr.append(obj_bb)
        label_img[indices] = labels 

        return bb_arr, pc_arr, label_img
    # no plane found -> return empty arrays
    return [], [], []

def get_objects_on_table(ros_pcd): #TODO remove target frame also from config
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

    eps = rospy.get_param("/get_objects_on_table/cluster_dbscan_eps")
    min_points = rospy.get_param("/get_objects_on_table/min_points")
    min_volume = rospy.get_param("/get_objects_on_table/min_volume")
    max_obj_height = rospy.get_param('/get_objects_on_table/max_obj_height')

    bb_arr, pc_arr, label_img = getter_of_objects_on_table(
        pcd, 
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
        '/get_objects_on_table/enable_rviz_visualization')
    if enable_rviz_visualization:
        pub = rospy.Publisher('objectsOnTableVisualizer',
                              MarkerArray, queue_size=10)

    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    o3d_bb_arr, o3d_pc_arr, _ = get_objects_on_table(req.point_cloud)
    ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
        o3d_bb_arr, base_frame, rospy.get_rostime())#TODO FIX FRAME

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
        '/get_objects_on_table/enable_rviz_visualization')
    if enable_rviz_visualization:
        pub = rospy.Publisher('objectsOnTableVisualizer',
                              MarkerArray, queue_size=10)

    base_frame = rospy.get_param("/table_plane_extractor/base_frame")
    o3d_bb_arr, o3d_pc_arr, _ = get_objects_on_table(req, base_frame)
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


def get_objects_on_table_server():
    '''
    Starts get_objects_on_table services
    Topics: /objects_on_table/get_bounding_boxes
            /objects_on_table/get_point_clouds
    '''
    rospy.init_node('get_objects_on_table')
    s_bb = rospy.Service('/objects_on_table/get_bounding_boxes',
                         GetBBOfObjectsOnTable, get_object_bbs)
    s_pc = rospy.Service('/objects_on_table/get_point_clouds',
                         GetPCOfObjectsOnTable, get_object_pcs)
    print("Ready to detect objects on table plane.")
    rospy.spin()


if __name__ == "__main__":
    try:
        get_objects_on_table_server()
    except rospy.ROSInterruptException:
        pass
