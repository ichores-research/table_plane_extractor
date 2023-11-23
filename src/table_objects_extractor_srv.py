#!/usr/bin/env python3
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
import numpy as np
import tf2_ros
import copy
from v4r_util.util import transformPointCloud, o3d_bb_list_to_ros_bb_arr
#from v4r_util.rviz_visualizer import rviz_visualizer
from table_plane_extractor.srv import GetBBOfObjectsOnTable, GetBBOfObjectsOnTableResponse, GetPCOfObjectsOnTable, GetPCOfObjectsOnTableResponse
from extractor_of_table_planes import extract_table_planes_from_pcd
from extractor_of_table_objects import extract_objects_from_tableplane
from vision_msgs.msg import BoundingBox3DArray

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from v4r_util.util import  o3d_bb_to_ros_bb
from vision_msgs.msg import BoundingBox3DArray
from itertools import chain

class rviz_visualizer:
    def __init__(self, topic="TablePlaneExtractorVisualizer"):
        self.pub = rospy.Publisher(
            topic, 
            MarkerArray, 
            queue_size=10)
        self.markers = {}

    def publish_ros_bb(self, ros_bb, namespace="", clear_old_markers=True):
        marker_arr = self.ros_bb_arr_to_rviz_marker_arr([ros_bb], namespace, clear_old_markers)
        self.pub.publish(marker_arr)

    def publish_ros_bb_arr(self, ros_bb_arr, namespace="", clear_old_markers=True):
        marker_arr = self.ros_bb_arr_to_rviz_marker_arr(ros_bb_arr, namespace, clear_old_markers)
        self.pub.publish(marker_arr)


    def publish_o3d_bb_arr(self, o3d_bb_arr, header, namespace="", clear_old_markers=True):
        ros_bb_arr = BoundingBox3DArray()
        ros_bb_arr.header = header
        ros_bb_arr.boxes = [o3d_bb_to_ros_bb(bb) for bb in o3d_bb_arr]
        marker_arr = self.ros_bb_arr_to_rviz_marker_arr(ros_bb_arr, namespace, clear_old_markers)
        self.pub.publish(marker_arr)

    def ros_bb_to_rviz_marker(self, ros_bb, namespace="", id=0, header=None):
        '''
        Converts BoundingBox3D to rviz Marker.
        Input: vision_msgs/BoundingBox3D ros_bb
        Output: visualization_msgs/Marker marker
        '''
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = rospy.get_rostime()
        marker.ns = namespace
        marker.id = id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose = ros_bb.center
        marker.scale = ros_bb.size
        marker.color.g = 1.0
        marker.color.a = 0.6
        return marker
    
    def clear_markers(self, namespace):
        '''
        Publishes a delete_all marker to clear all rviz markers with the given namespace.
        '''
        marker_arr = MarkerArray()
        marker = Marker()
        marker.ns = namespace
        marker.action = marker.DELETEALL
        marker_arr.markers.append(marker)

        del self.markers[namespace]
        l = [marker for marker_list in self.markers.values() for marker in marker_list.markers]
        marker_arr.markers += l
        self.pub.publish(marker_arr)

    def ros_bb_arr_to_rviz_marker_arr(self, ros_bb_arr, namespace, clear_old_markers=True):
        '''
        Converts BoundingBox3DArray into rviz MarkerArray. If clear_old_markers is set, a delete_all marker
        is added as the first marker so that old rviz markers get cleared.
        Input: vision_msgs/BoundingBox3DArray ros_bb_arr
            bool clear_old_markers
        Output: visualization_msgs/MarkerArray marker_arr
        '''
        # add delete_all as the first marker so that old markers are cleared
        marker_arr = MarkerArray()
        marker_arr.markers = []
        if clear_old_markers:
            if namespace in self.markers:
                self.clear_markers(namespace)

        # add marker for each detected object
        for i, obj in enumerate(ros_bb_arr.boxes):
            marker = self.ros_bb_to_rviz_marker(obj, namespace, i, ros_bb_arr.header)
            marker_arr.markers.append(marker)

        self.markers[namespace] = marker_arr

        return marker_arr

    
def table_objects_extractor(ros_pcd): #TODO remove target frame also from config
    '''
    Returns bounding boxes and pointclouds of objects found on a table plane.
    Output: list[open3d.geometry.OrientedBoundingBox] bb_arr
            list[open3d.geometry.PointCloud] pc_arr
            np.array label_img (flattened image with shape (width*height))
    '''
    
    base_frame = rospy.get_param("/table_plane_extractor/base_frame")
    enable_rviz_visualization = rospy.get_param(
        '/table_objects_extractor/enable_rviz_visualization')
    
    if enable_rviz_visualization:
        rviz_vis = rviz_visualizer('TablePlaneExtractorVisualizer')   

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

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
        copy.deepcopy(bboxes),
        eps, 
        min_points, 
        min_volume, 
        max_obj_height,
        height,
        width)
    
    if enable_rviz_visualization:
        rviz_vis.publish_o3d_bb_arr(bboxes, header, "table_plane")
        rviz_vis.publish_o3d_bb_arr(bb_arr, header, "objects_on_table")
        rospy.sleep(3.)
        rviz_vis.clear_markers("table_plane")

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
