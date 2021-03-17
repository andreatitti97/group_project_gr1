#ifndef _STRUCTURE_H_
#define _STRUCTURE_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
//#include "Drone.h"

using namespace std;
using namespace Eigen;


class StructureImpl;

class Structure
{
    private:
    //Puntatore alla classe impleentativa
    StructureImpl *structure_implementation; //mi riferisco alla classe PIDimpl tramite il puntatore *impl
     
    public:
    
    //Costruttore classe Panel
    Structure(Eigen::Vector2f structure_center_W, float theta, float size, float length);
    
    //Dichiarazione funzioni implementate nel file Structure.cpp
    //void init(float n_structure, float distance_between_structure,int configuration);

    void init(float n_strutture,float* x_waypoints, float* y_waypoints, float* orientations);
    
    //Function that pass variables from class Structure to StructureImpl
    void pass_to_class_GPS_error(float GPS_gamma, float GPS_eta, float GPS_delta);

    //Functions to obtain variables in file main from functions of class StructureImpl
    vector<float> obtain_waypoints_x_coo_gps_frame();
    vector<float> obtain_waypoints_y_coo_gps_frame();
    vector<float> obtain_waypoints_x_coo_world_frame();
    vector<float> obtain_waypoints_y_coo_world_frame();
    vector<float> obtain_xcoo_structure_world_frame();
    vector<float> obtain_ycoo_structure_world_frame();
    float obtain_theta();
    


    
     ~Structure();



};

#endif
