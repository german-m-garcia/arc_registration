## Before following this procedure..

At this point it is assumed that the data has been captured,
and the tool to manually align the views has been used
(this tool generates the transformation file between views,
as well as the extracted.pcd file with the object point clouds).

* epsom_salts/1/transformation.tf
* epsom_salts/1/camera_pose.tf
* epsom_salts/1/0000/extracted.pcd
* epsom_salts/1/0001/extracted.pcd



## folder structure of an object, e.g. epsom_salts:

epsom_salts/0
             /0000/
             /0001/
             /0002/
             [...]
           /1
             /0000/
             /0001/
             /0002/
             [...]


## point cloud files and transformation files

# epsom_salts/0
>
[c]          /cam_T_obj.tf: expresses the pose of the object the camera frame of reference
             [...]

# epsom_salts/1
>
[0]          /transformation.tf: contains the transfomation from view 0/ to view 1/
             /0000/...
[0]          /0001/extracted.pcd: contains the point cloud of the original frame, with the background points masked out
[1]          /0001/normalized.pcd: the point cloud of the object, normalized to the frame of reference of this view: 1/0000
[2]          /0001/normalized_to_0.pcd: the point cloud of the object, normalized to the frame of reference of the view: 0/0000
>
[1]          /0001/rotation.tf: 3x3 rotation matrix that expresses the pose of this view's object frame (view0) in the frame of the current view (viewj)
[2]          /0001/rotation_to_0.tf: 4x4 matrix that transforms 1/0001 to 0/0000
[2]          /0001/cam_T_view.tf: 3x3 matrix that expresses the pose of this view of the object in the camera frame of reference






## procedure to normalize the captured data


# [c]. calibrate the turntable in two steps

$ roslaunch arc_calibrate_turntable calibrate_table.launch
This will spin the table while running the SLAM method to give a first estimation of the pose of the table.
It will create two files:
* arc_calibrate_turntable/results/table.tf: contains the pose of the table in the camera frame of reference
* arc_calibrate_turntable/results/table.tf.txt: contains the name of the camera frame of reference

$ roslaunch arc_calibrate_turntable publish.launch
Here we visualize the obtained calibration and can refine it manually. It updates the following file:
* arc_calibrate_turntable/results/table.tf: contains the pose of the table in the camera frame of reference

# [0]. run the manual alignment plugin

This will create the following files:
* /0/transformation.tf
* /.../000i/extracted.pcd

# [1]. run register_views for each view of the captured object

$ rosrun arc_registration register_views /home/martin/Datasets/ARC/epsom_salts/0/
$ rosrun arc_registration register_views /home/martin/Datasets/ARC/epsom_salts/1/

This will create the following files:
* rotation.tf
* normalized.pcd
* camera_pose.tf

# [2]. run align_tfs_views for each view of the captured object

$ rosrun arc_registration align_tfs_views /home/martin/Datasets/ARC/epsom_salts/0/
$ rosrun arc_registration align_tfs_views /home/martin/Datasets/ARC/epsom_salts/1/

This will normalize the individual views to the frame of reference of view 0/0000. It will create
the following files:

* rotation_to_0.tf
* normalized_to_0.pcd

## to add grasps with the grasp_editor need to run
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 world model_cloud 30
$ rviz
