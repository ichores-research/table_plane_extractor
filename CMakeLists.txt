cmake_minimum_required(VERSION 3.0.2)
project(table_plane_extractor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  sensor_msgs
  vision_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  v4r_util
  robokudo_msgs
  actionlib
  grasping_pipeline_msgs
)


# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  TablePlaneExtractor.srv
#  GetPCOfObjectsOnTable.srv
#  GetBBOfObjectsOnTable.srv
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  sensor_msgs
#  vision_msgs
#  grasping_pipeline_msgs
#)


catkin_package()
include_directories()

