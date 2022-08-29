# table_plane_extractor

- Service for horizontal table plane extraction.  
- Service for getting objects (or multi-objects blobs if they are too close to each other) that are placed on the table plane.

## Dependencies ##
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
- [requirements.txt](requirements.txt)
- [CMakeLists.txt](CMakeLists.txt)

## Services

### TablePlaneExtractor
Table plane extractor who takes the point cloud as input and return possible horizontal planes with plane equation ( $x, y, z, d \rightarrow a x + b y + c z + d = 0$ ) and inlier cloud.

**Input:** sensor_msgs/PointCloud2 pointcloud  
**Output:** table_plane_extractor/Plane[] planes, sensor_msgs/PointCloud2[] clouds

```
src/table_plane_extractor_srv.py
```

### GetObjectsOnTable
Service that returns bounding boxes of objects found on a table plane.  
**Input:** sensor_msgs/PointCloud2 scene_pointcloud  
**Output:** vision_msgs/BoundingBox3DArray detected_objects
```
src/get_objects_on_table.py
```

## Startup

You can start the TablePlaneExtractor service with
```
roslaunch table_plane_extractor table_plane_extractor.launch
```
You can start the GetObjectsOnTable service with
```
roslaunch table_plane_extractor get_objects_on_table.launch
```

## Demo

You can find demo codes in the Files  
```
src/test_plane.py
src/objects_on_table_vis.py
```

## Status
stable, tested on Ubuntu 20.04 and ROS noetic.  


### Known Issues
- open3d only approximates the minimum volume bounding box -> for certain objects bounding box is not fitted perfectly.

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

```
sensor_msgs/PointCloud2 pointcloud
---
table_plane_extractor/Plane[] planes
sensor_msgs/PointCloud2[] clouds
```

### GetObjectsOnTable.srv

```
sensor_msgs/PointCloud2 scene_pointcloud  
---
vision_msgs/BoundingBox3DArray detected_objects
```


