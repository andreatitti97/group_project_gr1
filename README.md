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
TUORIAL FOR LAUNCH THE PROJECT WITH THE REAL DRONE.
NB: check the ros topics in the file ros_simulation_data for using the project with the drone or in simulation


Lancia il mondo e il drone in simulazione (Gazebo)
```
$ roslaunch cvg_sim_gazebo mytest.launch
 
```
Start the socket for receive images from the drone
```
$ rosrun drone_publish_image socketServerros2.py

```
Start the CV algorithm

```
$ rosrun group_project RGB_visual_ros_sim.py

```
Start the communication with the drone, check safety of the drone and of people in the environment befor this step

```
$ rosrun targ_pos_socket send_target_socket.py

```
