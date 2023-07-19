#!/usr/bin/python3

from math import atan, pi
import numpy as np
import open3d as o3d

def extract_table_planes_from_pcd(pcd, cluster_dbscan_eps = 0.15, min_cluster_size = 200, distance_threshold = 0.01, max_angle_deg = 5, z_min = 0.2):
    '''
    Extract possible horizontal planes from the point cloud.
    
    Parameters:
        pcd (open3d.geometry.PointCloud): Input point cloud without NANs and with Z pointing up.
        cluster_dbscan_eps (float): DBSCAN clustering epsilon value.
        min_cluster_size (int): Minimum number of points required to form a cluster.
        distance_threshold (float): RANSAC distance threshold for plane segmentation.
        max_angle_deg (float): Maximum angle (in degrees) allowed for a plane to be considered horizontal.
        z_min (float): Maximum Z coordinate value for a point to be considered as part of the floor.
        
    Returns:
        list: List of tuples (a, b, c, d) representing plane equations (a * x + b * y + c * z + d = 0).
        list: List of oriented bounding boxes (Open3D geometry objects) around the detected planes.
    '''
    
    planes = []
    bboxes = []

    max_angle_rad = max_angle_deg * pi/180

    points = np.array(pcd.points)
    floor = points[:, 2] < z_min
    pcd.points = o3d.utility.Vector3dVector(points[floor == 0])
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
            planes.append((a,b,c,d))


            print("Plane equation: {}x + {}y + {}z + {} = 0".format(a, b, c, d))
            bb_plane = plane_pc.get_minimal_oriented_bounding_box(robust=True)
            bboxes.append(bb_plane)
    return planes, bboxes