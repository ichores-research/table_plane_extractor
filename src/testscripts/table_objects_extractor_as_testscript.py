#!/usr/bin/env python3
import rospy
import actionlib
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
import message_filters
from sensor_msgs.msg import Image

class UseGetLabelImageOfObjectsOnTable():
    ''' Example of using the table_objects_extractor service. '''

    def __init__(self):
        self.cloud = None

        self.depth, self.rgb = None, None
        self.image_sub = self.__setup_image_subs(
            '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', 
            '/hsrb/head_rgbd_sensor/rgb/image_rect_color')

        self.timeout = 100

        rospy.init_node('table_objects_extractor_as_client')
        self.table_object_extractor = self.__setup_detector(
            '/table_objects_extractor/get_label_image', 
            self.timeout)
        print('wait for image messages')
        rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image) 
        rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image)

        try:
            goal = GenericImgProcAnnotatorGoal(rgb = self.rgb, depth = self.depth)
            result = self.get_estimator_result(goal)
        except rospy.ServiceException as e:
            print(e)

    def __setup_detector(self, detector_topic, timeout):
        rospy.loginfo('Waiting for table_object_extractor actionserver')
        
        table_object_extractor = actionlib.SimpleActionClient(
            detector_topic, 
            GenericImgProcAnnotatorAction)
        if not table_object_extractor.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to table_object_extractor \'{detector_topic}\' timed out!')
            raise TimeoutError
        return table_object_extractor

    def __setup_image_subs(self, depth_topic, rgb_topic):
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        image_sub = message_filters.ApproximateTimeSynchronizer([depth_sub, rgb_sub], 5, 0.1)
        image_sub.registerCallback(self.image_callback)
        return image_sub

    def pc_cb(self, data):
        self.cloud = data

    def get_estimator_result(self, goal):
        rospy.logdebug('Sending goal to estimator')
        self.table_object_extractor.send_goal(goal)

        rospy.logdebug('Waiting for estimator results')
        goal_finished = self.table_object_extractor.wait_for_result(rospy.Duration(self.timeout))
        if not goal_finished:
            rospy.logerr('table_object_extractor didn\'t return results before timing out!')
            raise TimeoutError
        result = self.table_object_extractor.get_result()
        rospy.loginfo(f'Detector detected {len(result.pose_results)} potential object poses.')

        return result




    def image_callback(self, depth, rgb):
        self.depth = depth
        self.rgb = rgb

if __name__ == "__main__":
    
    u = UseGetLabelImageOfObjectsOnTable()

