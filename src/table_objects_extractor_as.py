#! /usr/bin/env python3
import numpy as np
import rospy
import ros_numpy
import actionlib
from table_objects_extractor_srv import table_objects_extractor
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult
from table_plane_extractor.srv import GetBBOfObjectsOnTableRequest
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from v4r_util.depth_pcd import convert_ros_depth_img_to_pcd, convert_np_label_img_to_ros_color_img
from v4r_util.message_checks import check_for_rgb_depth


class GetObjectsOnTableAS():

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            '/table_objects_extractor/get_label_image', GenericImgProcAnnotatorAction, self.get_labels_img, False)

        self.enable_rviz_visualization = rospy.get_param(
            '/table_objects_extractor/enable_rviz_visualization')
        if self.enable_rviz_visualization:
            self.pub = rospy.Publisher('objectsOnTableLabelImage',
                                Image, queue_size=10)
        self.server.start()
        rospy.loginfo("TableObjectsExtractor: Actionserver started")

    def get_labels_img(self, goal):
        '''
        Actionserver that implements the GenericImgProcAnnotator message.

        The table-plane based object detector is not able to assign object classes to the
        detected objects. This means that all objects are in essence 'unknown'. This is encoded
        by assigning class_id = -1 and class_name = 'Unknown'. Additionally the label image is returned,
        which defines which pixel from the depth image belongs to which object.

        Topic: /objects_on_table/get_labels_img
        Expected Input: sensor_msgs/Image rgb, sensor_msgs/Image depth
        Returns: int32[] class_ids, string[] class_names, sensor_msgs/Image image
        '''
        goal_ok = check_for_rgb_depth(goal)
        if not goal_ok:
            self.server.set_aborted('Not every expected message field was passed to GetObjectsOnTableAS')
            return
        
        ros_cam_topic = rospy.get_param('/table_objects_extractor_as/cam_info_topic')
        cam_info = rospy.wait_for_message(ros_cam_topic, CameraInfo)

        #TODO due to not properly seperating the tablePlaneExtractor from ROS I have to do it this way
        # probably should refactor tablePlaneExtractor to have a method that is not dependant on ROS
        # and wrap this method with ROS specific stuff
        req = GetBBOfObjectsOnTableRequest()
        req.point_cloud, _ = convert_ros_depth_img_to_pcd(goal.depth, cam_info, project_valid_depth_only=False)
        o3d_bbs, _, labels = table_objects_extractor(req.point_cloud) 
        if labels is None:
            rospy.logerr("No objects extracted!")
            self.server.set_aborted(None)
            return
        
        np_label_img = labels.reshape(goal.depth.height, goal.depth.width)

        if self.enable_rviz_visualization:
            np_rgb_img = ros_numpy.numpify(goal.rgb)
            import cv2
            cv2.imwrite("Penis.jpg", np_label_img)
            print("Written image :3")
            color_img = convert_np_label_img_to_ros_color_img(np_label_img, np_rgb_img)
            self.pub.publish(color_img)

        res = GenericImgProcAnnotatorResult()
        unique_labels = np.unique(labels)
        detected_object_count = unique_labels[unique_labels != -1].shape[0]
        res.class_ids = [-1] * detected_object_count
        res.class_names = ['Unknown'] * detected_object_count
        res.pose_results = [Pose()] * detected_object_count
        res.image = ros_numpy.msgify(Image, np_label_img, encoding='16SC1')

        self.server.set_succeeded(res)

if __name__ == '__main__':
    rospy.init_node('table_objects_extractor_as')
    server = GetObjectsOnTableAS()
    rospy.spin()