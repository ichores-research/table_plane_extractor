# table_plane_extractor

Service for horizontal table plane extraction

## Dependencies ##
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
- [Open3d](http://www.open3d.org/docs/release/)
- [open3d_ros_helper](https://github.com/SeungBack/open3d-ros-helper) 
- [tf2_ros](http://wiki.ros.org/tf2_ros)

## Service

Table plane extractor who takes the point cloud topic as input and return possible horizontal planes with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and inlier cloud.
Input: string pointcloud_topic
Output: table_plane_extractor/Plane[] planes, sensor_msgs/PointCloud2[] clouds
```
src/table_plane_extractor_srv.py
```

## Demo

You can find a demo code in the File 
```
src/test_plane.py
```

## Startup

You can start the service with
```
roslaunch table_plane_extractor table_plane_extractor.launch
```

## Message

### Plane.msg

#### Fields
```
float32 x
float32 y
float32 z
float32 d
```

## Service

### TablePlaneExtractor.srv

#### Goal
```
string pointcloud_topic
```

#### Result
```
table_plane_extractor/Plane[] planes
sensor_msgs/PointCloud2[] clouds
```
