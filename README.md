# group_project_gr1
REPOSITORY FOR THE GROUP PROJECT - Robotics Engineering UNIGE


## Requirements
* ROS kinetic
* Ubuntu 16.04

The package ardrone-autonomy might be required and can be installed with the following command on the shell


```
sudo apt-get install ros-kinetic-ardrone-autonomy
sudo apt-get install ros-kinetic-hector-*
```

## Launching commands

```
$ roslaunch cvg_sim_gazebo mytest.launch
$ roslaunch box_urdf spawn_camera_box_model.launch 
$ roslaunch box_urdf spawn_model.launch
$ rosrun group_project drone_control
$ rosrun group_project RGB_visual_ros_sim.py 
```


