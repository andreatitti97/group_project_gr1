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
 
```
In primo momento in una nuova shell viene lanciato il nodo relativo al controllo del drone, successivamente (dopo 40 secondi) una nuova shell viene lancciata col nodo relativo all'algoritmo di OpenCV.
  
