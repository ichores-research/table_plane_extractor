# table_plane_extractor
Service for horizontal table plane extraction.  
Service for getting objects that are placed on the table plane.

## Dependencies ##
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
- [requirements.txt](requirements.txt)
- [package.xml](package.xml)
- [CMakeLists.txt](CMakeLists.txt)

## Services

### TablePlaneExtractor
Table plane extractor who takes the point cloud as input. Returns possible horizontal planes with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and bounding boxes around the planes.

**Service topic:** 
```
/test/table_plane_extractor
```
**Input:** 
```
sensor_msgs/PointCloud2 pointcloud
```
**Result:** 
```
table_plane_extractor/Plane[] planes, 
vision_msgs/BoundingBox3DArray plane_bounding_boxes
```
**File:**
```
src/table_plane_extractor_srv.py
```

### GetObjectsOnTable
Service that returns bounding boxes of objects found on a table plane. 

**Input:**
```
sensor_msgs/PointCloud2 scene_pointcloud
```
**Result:**
```
vision_msgs/BoundingBox3DArray detected_objects
```
**File:**
```
src/get_objects_on_table.py
```

## Test

You can test and see example usages of the services with the following scripts
```
src/test_plane.py
src/objects_on_table_vis.py
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

## Status
stable, tested on Ubuntu 20.04 and ROS noetic.  
Known Issue: open3d only approximates the minimum volume bounding box -> for certain objects bounding box is not fitted perfectly.

## Message

### Plane.msg

#### Fields
```
float32 x
float32 y
float32 z
float32 d
```

