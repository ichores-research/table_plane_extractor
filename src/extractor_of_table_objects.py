import numpy as np
from v4r_util.util import get_minimum_oriented_bounding_box


def extract_objects_from_tableplane(pcd, table_planes, eps, min_points, min_volume, max_obj_height, height, width):
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
    # no plane found -> return Nones
    return None, None, None