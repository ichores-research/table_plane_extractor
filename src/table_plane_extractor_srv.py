#!/usr/bin/python3

from table_plane_extractor.srv import TablePlaneExtractor, TablePlaneExtractorResponse
from table_plane_extractor.msg import Plane
import numpy as np
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as orh
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

def transformPointCloud(cloud, target_frame, source_frame, tf_buffer):
    ''' 
    Transform pointcloud from source_frame to target_frame
    Input: sensor_msgs/PointCloud2 cloud, string target_frame, string source_frame
            tf2_ros.Buffer tf_buffer
    Output: sensor_msgs/PointCloud2 transformedCloud
    '''
    
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        except:
            continue
        transformedCloud = do_transform_cloud(cloud, transform)
        return transformedCloud

def table_plane_extractor_methode(req):
    ''' 
    Table plane extractor who takes the point cloud as input and return possible horizontal planes 
    with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and inlier cloud.
    Everything is transformed and relative to the 'map' frame_id.
    Input: sensor_msgs/PointCloud2 inputCloud
    Output: table_plane_extractor/Plane[] planes, sensor_msgs/PointCloud2[] clouds
    '''
    planes = []
    cloudes = []
    
    #tf_listener = tf.TransformListener()
    # tf buffer for tf transform
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
    pcd = req.point_cloud
    header = pcd.header
    pcd = transformPointCloud(pcd, "map", pcd.header.frame_id, tf_buffer)
    pcd_orig = pcd
    pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

    #downsample cloud
    downsample_vox_size = rospy.get_param("/downsample_vox_size")
    pcd = pcd.voxel_down_sample(voxel_size=downsample_vox_size)

    #compute normals and filter out any points that are not on horizontal area
    normals_search_radius = rospy.get_param("/normals_search_radius")
    normals_kNN = rospy.get_param("/normals_kNN")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=normals_search_radius, max_nn=normals_kNN))
    normals = np.asarray(pcd.normals)
    normals_thresh = rospy.get_param("/normals_thresh")
    pcd = pcd.select_by_index(np.where((abs(normals[:, 0]) < normals_thresh) 
                                                    & (abs(normals[:, 1]) < normals_thresh))[0])
    scene = pcd

    size_cof = len(pcd.points)
    outlier_cloud = pcd
    h_planes = []  #list of horizontal planes (parameters)
    h_plane_clouds = [] #list of horizontal plane pointclouds, not clustered

    scene_filtered = scene
    big_clusters_left = True
    cluster_dbscan_minpoints = rospy.get_param("/cluster_dbscan_minpoints")
    cluster_dbscan_eps = rospy.get_param("/cluster_dbscan_eps")
    min_cluster_size = rospy.get_param("/min_cluster_size")
    #get planes with RANSAC
    while len(outlier_cloud.points)>cluster_dbscan_minpoints and big_clusters_left:
        distance_threshold = rospy.get_param("/plane_segmentation_distance_threshold")
        plane_model, inliers = outlier_cloud.segment_plane(distance_threshold=distance_threshold,
                                                ransac_n=3,
                                                num_iterations=1000)
        [a, b, c, d] = plane_model
        planes.append(Plane(a, b, c,d))
        print("Plane equation: {}x + {}y + {}z + {} = 0".format(a,b,c,d))
        inlier_cloud = outlier_cloud.select_by_index(inliers)
        cloudes.append(orh.o3dpc_to_rospc(inlier_cloud, 'map', header.stamp))
        outlier_cloud = outlier_cloud.select_by_index(inliers, invert=True)
        
        #break if there are only small clusters left
        idx = outlier_cloud.cluster_dbscan(cluster_dbscan_eps, cluster_dbscan_minpoints)
        values = np.unique(idx)
        for c in values:
            #select clustered plane cloud
            cluster_idx = np.where(np.asarray(idx) == c)
            if len(cluster_idx[0]) > min_cluster_size:
                big_clusters_left = True
                break
            else:
                big_clusters_left = False

    return TablePlaneExtractorResponse(planes, cloudes)

def table_plane_extractor_server():
    ''' 
    Starting table plane extractor server node
    Topic: /test/table_plane_extractor
    '''
    rospy.init_node('table_plane_extractor_server')
    s = rospy.Service('/test/table_plane_extractor', TablePlaneExtractor, table_plane_extractor_methode)
    print("Ready to extract planes")
    rospy.spin()

if __name__ == "__main__":
    table_plane_extractor_server()