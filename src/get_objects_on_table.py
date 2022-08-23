#!/usr/bin/env python3
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import copy
import rospy
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from scipy.spatial.transform import Rotation as R
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
from table_plane_extractor.srv import GetObjectsOnTable, GetObjectsOnTableResponse
from table_plane_extractor_srv import transformPointCloud, table_plane_extractor_methode

target_frame = 'base_link'

def get_objects_on_table(req):
    '''
    Service that returns bounding boxes of objects found on a table plane.
    Input: sensor_msgs/PointCloud2 scene_pointcloud
    Output: vision_msgs/BoundingBox3DArray detected_objects
    '''
    try:
        # call directly as it's in the same package -> no ros overhead
        response = table_plane_extractor_methode(req)
    except Exception as e:
        rospy.logerr(e)
        raise rospy.ServiceException(e)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    scene_rel_to_camera = req.point_cloud
    camera_frame_id = scene_rel_to_camera.header.frame_id


    bb_arr = BoundingBox3DArray()
    bb_arr.header.stamp = scene_rel_to_camera.header.stamp
    bb_arr.header.frame_id = target_frame
    bb_arr.boxes = []
    for plane_cloud in response.clouds:
        plane_o3d = orh.rospc_to_o3dpc(plane_cloud)
        bb_cloud = plane_o3d.get_oriented_bounding_box()
        # remove floor (plane is in map frame)
        z_min = rospy.get_param('/get_objects_on_table/z_min')
        if bb_cloud.center[2] < z_min:
            continue
        
        # transform scene pointcloud to map frame
        scene = transformPointCloud(scene_rel_to_camera, plane_cloud.header.frame_id, camera_frame_id, tf_buffer)
        scene_o3d = orh.rospc_to_o3dpc(scene)

        # get bounding box above table
        bb_above_table = copy.deepcopy(bb_cloud)
        bb_above_table.center = (bb_above_table.center[0], bb_above_table.center[1], bb_above_table.center[2]+0.2+bb_above_table.extent[2])
        bb_above_table.extent = (bb_above_table.extent[0]+0.04, bb_above_table.extent[1]+0.04, bb_above_table.extent[2]+0.4)
        
        # filter out points that are not above the table
        indices = np.array(bb_above_table.get_point_indices_within_bounding_box(scene_o3d.points))
        scene_above_table = scene_o3d.select_by_index(indices)

        # transform pointcloud to base-link-frame (for haf-grasping)
        scene_above_table_ros = orh.o3dpc_to_rospc(scene_above_table)
        scene_above_table_ros = transformPointCloud(scene_above_table_ros, target_frame, plane_cloud.header.frame_id, tf_buffer)
        scene_above_table = orh.rospc_to_o3dpc(scene_above_table_ros)
        
        # segment pointcloud into objects
        eps = rospy.get_param("/get_objects_on_table/cluster_dbscan_eps")
        min_points = rospy.get_param("/get_objects_on_table/min_points")
        labels = np.array(scene_above_table.cluster_dbscan(eps=eps, min_points=min_points))
        labels_unique = np.unique(labels)

        # get bounding box for each object
        for label in labels_unique:
            if label == -1:
                continue
            obj_indices = np.where(labels == label)
            obj = scene_above_table.select_by_index(obj_indices[0])
            obj_bb = obj.get_oriented_bounding_box()
            # filter very small bounding boxes
            min_volume = rospy.get_param("/get_objects_on_table/min_volume")
            if obj_bb.volume() < min_volume:
                continue
            obj_bb_ros = o3d_bb_to_ros_bb(obj_bb)
            bb_arr.boxes.append(obj_bb_ros)
        rospy.loginfo("Detected " + str(len(bb_arr.boxes)) + " objects!")
        return GetObjectsOnTableResponse(bb_arr)
    # return empty array
    return GetObjectsOnTableResponse(bb_arr)


def o3d_bb_to_ros_bb(o3d_bb):
    '''
    Converts open3d OrientedBoundingBox to ros BoundingBox3D.
    Input: open3d.geometry.OrientedBoundingBox o3d_bb
    Output: vision_msgs/BoundingBox3D ros_bb
    '''
    rot = R.from_matrix(copy.deepcopy(o3d_bb.R))
    quat = rot.as_quat()

    ros_bb = BoundingBox3D()
    ros_bb.center.position.x = o3d_bb.center[0]
    ros_bb.center.position.y = o3d_bb.center[1]
    ros_bb.center.position.z = o3d_bb.center[2]
    ros_bb.center.orientation.x = quat[0]
    ros_bb.center.orientation.y = quat[1]
    ros_bb.center.orientation.z = quat[2]
    ros_bb.center.orientation.w = quat[3]
    ros_bb.size.x = o3d_bb.extent[0]
    ros_bb.size.y = o3d_bb.extent[1]
    ros_bb.size.z = o3d_bb.extent[2]
    
    return ros_bb


def get_objects_on_table_server():
    '''
    Starts get_objects_on_table service
    Topic: /objects_on_table/get_bounding_boxes
    '''
    rospy.init_node('get_objects_on_table')
    s = rospy.Service('/objects_on_table/get_bounding_boxes', GetObjectsOnTable, get_objects_on_table)
    print("Ready to detect objects on table plane.")
    rospy.spin()

if __name__ == "__main__":
    try:
        get_objects_on_table_server()
    except rospy.ROSInterruptException:
        pass
