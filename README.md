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
## TO DO LIST
- Creare un URDF per il modello dei pannelli (bianchi) e un corrispondente lanch file, verificando che funzioni con la simulazione  
  per ora ho inserito manualmente un pannello su Gazebo per testare l'algoritmo.(DONE)  
  Per aprire la simulazione con il nuovo environment scrivere:  
  ```
	roslaunch cvg_sim_gazebo mytest.launch
  ```
  Oppure se si vuole aprire la simulazione con un mondo specifico:  
  ```
	roslaunch cvg_sim_gazebo mytest.launch wname:=(Cartella in cui è contenuto)/(nome del mondo).world
  ```
 Con "(Cartella in cui è contenuto)" che può essere o "/wolds" (dove è contenuto il mondo appena sviluppato) o "/worlds_beckup" (dove è contenuto empty.world)
- Testare l'algoritmo di navigazione in scenari diversi, per ora è stato testato su un pannelo bianco di grandi dimensioni, il drone si       solleva, lo rileva e viaggia sul pannello fino al capo opposto per poi girarsi e tornare al capo di partenza ( da cui in poi va dritto seguendo sempre la GPS line). 
- Utilizzare joint rotazionali per le camere (gimbal) 
- Creare un URDF con un environment "finale" in cui far lavorare il robot (dopo i test sulla navigazione)
- Test finale della navigazione sul environment finale
- 
-  
