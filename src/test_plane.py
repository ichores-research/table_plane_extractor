#!/usr/bin/python3

from table_plane_extractor.srv import TablePlaneExtractor
from visualization_msgs.msg import MarkerArray
import rospy
from sensor_msgs.msg import PointCloud2
from util import ros_bb_arr_to_rviz_marker_arr


class UseTablePlaneExtractor():
    ''' Example of using the table plane extractor service. '''

    def __init__(self):
        self.cloud = None
        # get point cloud
        topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        sub = rospy.Subscriber(topic, PointCloud2, self.pc_cb)
        pub = rospy.Publisher('PlaneBoundingBoxVisualizer', MarkerArray, queue_size=10)
        print("waiting for pointcloud message")
        rospy.wait_for_message(topic, PointCloud2)
        print("waiting for table_plane_extractor service")
        rospy.wait_for_service('/test/table_plane_extractor')
        try:
            # use service
            print("calling service")
            start = rospy.get_time()
            table_extractor = rospy.ServiceProxy('/test/table_plane_extractor', TablePlaneExtractor)
            response = table_extractor(self.cloud)
            # publish markers
            marker_arr = ros_bb_arr_to_rviz_marker_arr(response.plane_bounding_boxes)
            pub.publish(marker_arr)
            print("done in " + str(rospy.get_time() - start) + " s")

        except rospy.ServiceException as e:
            print(e)

    def pc_cb(self, data):
        self.cloud = data

if __name__ == "__main__":
    rospy.init_node('table_plane_extractor_client')
    u = UseTablePlaneExtractor()