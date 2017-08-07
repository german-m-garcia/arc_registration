## Instructions for capturing data with the turntable


#checklist
* Nikon camera switched on?
* SR300 and Nikon connected to USB3?
* turntable switched on?
* LED panels on?
* unmount the Nikon camera from the file system 


#start sr300
roscd arc_launch
roslaunch  hardware/sr300.launch

#start the turntable
rosrun arc_turntable turntable_camera

#mount bc
sudo mount bigcuda:/home/cache/arc /bc5/

# start rqt
rqt

Steps on the ARC/Turntable capture plugin:
* place object and stop the turntable
* autofocus on and off
* remove the object
* capture background button without a name for the object entry


# background substraction
rosrun arc_collection_manager  collection_manager  /bc5 /bc5/competition_set.txt
