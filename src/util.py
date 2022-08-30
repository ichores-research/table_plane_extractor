import copy
import open3d as o3d
from vision_msgs.msg import BoundingBox3D
from scipy.spatial.transform import Rotation as R
import rospy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray

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

def ros_bb_arr_to_rviz_marker_arr(ros_bb_arr, clear_old_markers=True):
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
        marker_delete_all = Marker()
        marker_delete_all.action = marker_delete_all.DELETEALL
        marker_arr.markers.append(marker_delete_all)
    id = 0
    # add marker for each detected object
    for obj in ros_bb_arr.boxes:
        marker = Marker()
        marker.header.frame_id = ros_bb_arr.header.frame_id
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "ObjectsOnTable"
        marker.id = id
        id = id + 1
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose = obj.center
        marker.scale = obj.size
        marker.color.g = 1.0
        marker.color.a = 0.6
        marker_arr.markers.append(marker)
    return marker_arr

