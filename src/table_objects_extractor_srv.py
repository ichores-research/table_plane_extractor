#!/usr/bin/env python3
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
import numpy as np
import tf2_ros
import copy
from v4r_util.util import transformPointCloud, o3d_bb_list_to_ros_bb_arr
from v4r_util.rviz_visualizer import RvizVisualizer
from table_plane_extractor.srv import GetBBOfObjectsOnTable, GetBBOfObjectsOnTableResponse, GetPCOfObjectsOnTable, GetPCOfObjectsOnTableResponse
from extractor_of_table_planes import extract_table_planes_from_pcd
from extractor_of_table_objects import extract_objects_from_tableplane
from vision_msgs.msg import BoundingBox3DArray


def table_objects_extractor(pcd): #TODO remove target frame also from config
    '''
    Returns bounding boxes and pointclouds of objects found on a table plane.
    Output: list[open3d.geometry.OrientedBoundingBox] bb_arr
            list[open3d.geometry.PointCloud] pc_arr
            np.array label_img (flattened image with shape (width*height))
    '''
    
    object_params = rospy.get_param("table_objects_extractor")
    table_params = rospy.get_param("table_plane_extractor")
    
    if table_params["enable_rviz_visualization"]:
        rviz_vis = RvizVisualizer('TablePlaneExtractorVisualizer')   

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    height = pcd.height
    width = pcd.width
    
    #make sure pointcloud has z pointing up
    pcd = transformPointCloud(pcd, table_params['base_frame'], pcd.header.frame_id, tf_buffer) 

    header = pcd.header

    pcd_with_nans = orh.rospc_to_o3dpc(pcd, remove_nans=False) #TODO why does tableplaneextractor die with nans
    pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

    # downsample cloud
    pcd_downsampled = pcd.voxel_down_sample(
        voxel_size=table_params["downsample_vox_size"])

    _, bboxes = extract_table_planes_from_pcd(
        pcd_downsampled, 
        table_params["cluster_dbscan_eps"], 
        table_params["min_cluster_size"], 
        table_params["plane_segmentation_distance_threshold"], 
        table_params["max_angle_deg"],
        table_params["z_min"])

    bb_arr, pc_arr, label_img = extract_objects_from_tableplane(
        pcd_with_nans, 
        copy.deepcopy(bboxes),
        object_params["cluster_dbscan_eps"], 
        object_params["min_points"], 
        object_params["min_volume"], 
        object_params["max_obj_height"],
        height,
        width)
    
    if table_params["enable_rviz_visualization"]:
        rviz_vis.publish_o3d_bb_arr(bboxes, header, "table_plane")
        rviz_vis.publish_o3d_bb_arr(bb_arr, header, "objects_on_table")
        rospy.sleep(3.)
        rviz_vis.clear_markers("plane")

    return bb_arr, pc_arr, label_img


def get_object_bbs(req):
    '''
    Service that returns bounding boxes of objects found on a table plane
    Topic: /objects_on_table/get_bounding_boxes
    Input: sensor_msgs/PointCloud2 scene
    Output: vision_msgs/BoundingBox3DArray detected_objects
    '''

    base_frame = rospy.get_param("/table_plane_extractor/base_frame")
    
    o3d_bb_arr, _, _ = table_objects_extractor(req.point_cloud)
    ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
        o3d_bb_arr, base_frame, rospy.get_rostime())

    return GetBBOfObjectsOnTableResponse(ros_bb_arr)


def get_object_pcs(req):
    '''Service that returns the pointclouds of objects found on a table plane
        Topic: /objects_on_table/get_point_clouds
        Input: sensor_msgs/PointCloud2 scene
        Output: sensor_msgs/PointCloud2[] detected_objects
    '''
    base_frame = rospy.get_param("/table_plane_extractor/base_frame")

    _, o3d_pc_arr, _ = table_objects_extractor(req.point_cloud)

    ros_pc_arr = []
    for pc in o3d_pc_arr:
        ros_pc = orh.o3dpc_to_rospc(pc, base_frame, rospy.get_rostime())
        ros_pc_arr.append(ros_pc)

    return GetPCOfObjectsOnTableResponse(ros_pc_arr)


def table_objects_extractor_server():
    '''
    Starts table_objects_extractor services
    Topics: /table_objects_extractor/get_bounding_boxes
            /table_objects_extractor/get_point_clouds
    '''
    rospy.init_node('table_objects_extractor')
    s_bb = rospy.Service('/table_objects_extractor/get_bounding_boxes',
                         GetBBOfObjectsOnTable, get_object_bbs)
    s_pc = rospy.Service('/table_objects_extractor/get_point_clouds',
                         GetPCOfObjectsOnTable, get_object_pcs)
    print("Ready to detect objects on table plane.")
    rospy.spin()


if __name__ == "__main__":
    try:
        table_objects_extractor_server()
    except rospy.ROSInterruptException:
        pass
