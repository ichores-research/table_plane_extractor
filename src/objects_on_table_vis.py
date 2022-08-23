#!/usr/bin/env python3
import rospy
from vision_msgs.msg import BoundingBox3DArray
from table_plane_extractor.srv import GetObjectsOnTable
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

class visualizeObjectsOnTable():
    ''' Example usage of get_objects_on_table service. '''
    def __init__(self):
        rospy.init_node('objects_on_table_visualizer', anonymous=True)
        self.cloud = None
        self.pointcloud_sub = rospy.Subscriber(topic, PointCloud2, self.pointcloud_cb)
        rospy.wait_for_message(topic, PointCloud2)

        pub = rospy.Publisher('objectsOnTableVisualizer', MarkerArray, queue_size=3)
        rospy.wait_for_service('/objects_on_table/get_bounding_boxes')
        s = rospy.ServiceProxy('/objects_on_table/get_bounding_boxes', GetObjectsOnTable)
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            try:
                res = s(self.cloud)
                # add delete_all as the first marker so that old markers are cleared
                marker_delete_all = Marker()
                marker_delete_all.action = marker_delete_all.DELETEALL
                marker_arr = MarkerArray()
                marker_arr.markers = []
                marker_arr.markers.append(marker_delete_all)
                id = 0
                # add marker for each detected object
                for obj in res.detected_objects.boxes:
                    marker = Marker()
                    marker.header.frame_id = res.detected_objects.header.frame_id
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
                pub.publish(marker_arr)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                rate.sleep()
                continue
            rate.sleep()

    def pointcloud_cb(self, data):
        self.cloud = data

if __name__ == '__main__':
    try:
        visualizeObjectsOnTable()
    except rospy.ROSInterruptException:
        pass
