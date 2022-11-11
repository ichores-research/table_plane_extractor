#!/usr/bin/env python3
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
import numpy as np
import tf2_ros
from visualization_msgs.msg import MarkerArray
from v4r_util.util import ros_bb_to_o3d_bb, transformPointCloud, o3d_bb_list_to_ros_bb_arr, ros_bb_arr_to_rviz_marker_arr, get_minimum_oriented_bounding_box
from table_plane_extractor.srv import GetBBOfObjectsOnTable, GetBBOfObjectsOnTableResponse, GetPCOfObjectsOnTable, GetPCOfObjectsOnTableResponse
from table_plane_extractor_srv import table_plane_extractor_methode


def get_objects_on_table(req, target_frame):
    '''
    Returns bounding boxes and pointclouds of objects found on a table plane.
    Input: Request req (with point_cloud attribute)
           string target_frame
    Output: list[open3d.geometry.OrientedBoundingBox] bb_arr
            list[open3d.geometry.PointCloud] pc_arr
    '''
    try:
        # call directly as it's in the same package -> no ros overhead
        response = table_plane_extractor_methode(req)
    except Exception as e:
        rospy.logerr(e)
        raise rospy.ServiceException(e)

    response_header = response.plane_bounding_boxes.header
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    scene_rel_to_camera = req.point_cloud
    camera_frame_id = scene_rel_to_camera.header.frame_id

    pc_arr = []
    bb_arr = []

    eps = rospy.get_param("/get_objects_on_table/cluster_dbscan_eps")
    min_points = rospy.get_param("/get_objects_on_table/min_points")
    min_volume = rospy.get_param("/get_objects_on_table/min_volume")
    max_obj_height = rospy.get_param('/get_objects_on_table/max_obj_height')

    for plane_bb in response.plane_bounding_boxes.boxes:
        # transform scene pointcloud to table_plane_extractor frame
        scene = transformPointCloud(
            scene_rel_to_camera, response_header.frame_id, camera_frame_id, tf_buffer)
        scene_o3d = orh.rospc_to_o3dpc(scene)

        # get bounding box above table
        bb_above_table = ros_bb_to_o3d_bb(plane_bb)
        bb_above_table.center = (bb_above_table.center[0], bb_above_table.center[1],
                                 bb_above_table.center[2]+max_obj_height/2.0+bb_above_table.extent[2])
        bb_above_table.extent = (
            bb_above_table.extent[0]+0.04, bb_above_table.extent[1]+0.04, bb_above_table.extent[2]+max_obj_height)

        # filter out points that are not above the table
        indices = np.array(
            bb_above_table.get_point_indices_within_bounding_box(scene_o3d.points))
        scene_above_table = scene_o3d.select_by_index(indices)

        # transform pointcloud to base-link-frame (for haf-grasping)
        scene_above_table_ros = orh.o3dpc_to_rospc(scene_above_table)
        scene_above_table_ros = transformPointCloud(
            scene_above_table_ros, target_frame, response_header.frame_id, tf_buffer)
        scene_above_table = orh.rospc_to_o3dpc(scene_above_table_ros)

        # segment scene-pointcloud into objects
        labels = np.array(scene_above_table.cluster_dbscan(
            eps=eps, min_points=min_points))
        labels_unique = np.unique(labels)

        # get bounding box and pointcloud for each object
        for label in labels_unique:
            if label == -1:
                continue
            obj_indices = np.where(labels == label)
            obj = scene_above_table.select_by_index(obj_indices[0])
            obj_bb = get_minimum_oriented_bounding_box(obj)
            # filter very small bounding boxes
            if obj_bb.volume() < min_volume:
                continue
            pc_arr.append(obj)
            bb_arr.append(obj_bb)
        return bb_arr, pc_arr
    # return empty array
    return bb_arr, pc_arr


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

    target_frame = rospy.get_param('/get_objects_on_table/target_frame')
    o3d_bb_arr, o3d_pc_arr = get_objects_on_table(req, target_frame)
    ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
        o3d_bb_arr, target_frame, rospy.get_rostime())

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

    target_frame = rospy.get_param('/get_objects_on_table/target_frame')
    o3d_bb_arr, o3d_pc_arr = get_objects_on_table(req, target_frame)
    ros_pc_arr = []
    for pc in o3d_pc_arr:
        ros_pc = orh.o3dpc_to_rospc(pc, target_frame, rospy.get_rostime())
        ros_pc_arr.append(ros_pc)

    if enable_rviz_visualization:
        ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
            o3d_bb_arr, target_frame, rospy.get_rostime())
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
