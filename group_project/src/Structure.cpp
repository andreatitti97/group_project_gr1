#ifndef _PANEL_SOURCE_H_
#define _PANEL_SOURCE_H_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include "group_project/pid.h"
#include "group_project/Structure.h"
#include "group_project/KF.h"
//#include "/home/lucamora/catkin_ws/src/solar_structure_project/include/solar_structure_project/waypoints.h"
//#include "Drone.h"

using namespace std;
using namespace Eigen;



class StructureImpl
{
    private:
    //Center coordinate in world frame
    vector <float> xcenter_structure_world_frame;
    vector <float> ycenter_structure_world_frame;

    //Center coordinates in GPS frame
    vector <float> xcenter_structure_frame;
    vector <float> ycenter_structure_frame;

    //Coo P1 P2 in structure frame
    vector <float> xcoo_structure_frame; //Contiene punti P1 P2 per ciascun pannello nell'ordine P1 P2 P2 P1 ecc
    vector <float> ycoo_structure_frame;

    //Coordinates P1, P2 point in world frame
    vector <float> xcoo_structure_world_frame; //Contiene punti P1 P2 per ciascun pannello nell'ordine P1 P2 P2 P1 ecc
    vector <float> ycoo_structure_world_frame;

    //Variabili da passare alla classe WayPoints relative alle coordinate GPS dei punti di target
     vector<float> waypoints_x_coo_gps_frame;
     vector<float> waypoints_y_coo_gps_frame;
     vector<float> waypoints_x_coo_world_frame;
     vector<float> waypoints_y_coo_world_frame;


    //Variabili inizializzate con costruttore
    float _theta;
    float _length;
    float _size;
    Eigen::Vector2f _structure_center_W;

    //GPS Variables
    float _GPS_delta;
    float _GPS_gamma;
    float _GPS_eta;


    //Declaration of Private functions
    void place_structure_start_end_point(float n_structure, float distance_between_structures,int configuration); //Vanno passati elementi che non sono parte della classe
    void transformation_structure_center_from_body_to_world();
    void transformation_point_from_structure_to_world_frame();
    void transformation_GPS_point_from_GPS_to_world_frame();

    public:
    float x_center;
    float y_center;
    float P1_x;
    float P1_y;
    float P2_x;
    float P2_y;




    //Class constructor, prende i termini dati in input alla class PID
    StructureImpl(Eigen::Vector2f structure_center_W, float theta, float size, float length);

    void init(float n_structure, float distance_between_structures,int configuration);
    void pass_to_class_GPS_error(float GPS_gamma, float GPS_eta, float GPS_delta);

    //Function to obtain variables from class StructureImpl in main
    vector<float> obtain_waypoints_x_coo_gps_frame();
    vector<float> obtain_waypoints_y_coo_gps_frame();
    vector<float> obtain_waypoints_x_coo_world_frame();
    vector<float> obtain_waypoints_y_coo_world_frame();
    vector<float> obtain_xcoo_structure_world_frame();
    vector<float> obtain_ycoo_structure_world_frame();
    float obtain_theta();

    //Destructor
     ~StructureImpl()
    {

    }

};

//Class StructureImpl call from header
Structure::Structure(Eigen::Vector2f structure_center_W, float theta, float size, float length)
{
    structure_implementation = new StructureImpl(structure_center_W, theta, size, length);
}

//Function dcall
void Structure::init(float n_structure, float distance_between_structures,int configuration)
{
    return structure_implementation -> init(n_structure, distance_between_structures, configuration);
}

void Structure::pass_to_class_GPS_error(float GPS_gamma, float GPS_eta, float GPS_delta)
{
     return structure_implementation -> pass_to_class_GPS_error(GPS_gamma, GPS_eta, GPS_delta);
}

vector<float> Structure::obtain_waypoints_x_coo_gps_frame()
{
    return structure_implementation -> obtain_waypoints_x_coo_gps_frame();
}

vector<float> Structure::obtain_waypoints_y_coo_gps_frame()
{
    return structure_implementation -> obtain_waypoints_y_coo_gps_frame();
}

vector<float> Structure::obtain_waypoints_x_coo_world_frame()
{
    return structure_implementation -> obtain_waypoints_x_coo_world_frame();
}

vector<float> Structure::obtain_waypoints_y_coo_world_frame()
{
    return structure_implementation -> obtain_waypoints_y_coo_world_frame();
}

vector<float> Structure::obtain_xcoo_structure_world_frame()
{
    return structure_implementation -> obtain_xcoo_structure_world_frame();
}

vector<float> Structure::obtain_ycoo_structure_world_frame()
{
    return structure_implementation -> obtain_ycoo_structure_world_frame();
}

float Structure::obtain_theta()
{
    return structure_implementation -> obtain_theta();
}

//Il distruttore dealloca anche la classe Panel
Structure::~Structure()
{
    delete structure_implementation;
}


/* Implementation */

StructureImpl::StructureImpl(Eigen::Vector2f structure_center_W, float theta, float size, float length ):
//Definisco TUTTE le variabiliche verranno utilizzate nelle funzioni e definite in StructureImpl
//Le variabili passate con costruttore devono essere rinominate e passate al file cpp definendole in private
    _structure_center_W(structure_center_W),
    _theta(theta),
    _size(size),
    _length(length),
    xcenter_structure_world_frame(), //inizializza un vettore con nessun elemento
    ycenter_structure_world_frame(),
    xcenter_structure_frame(),
    ycenter_structure_frame(),
    xcoo_structure_frame(),
    ycoo_structure_frame(),
    xcoo_structure_world_frame(),
    ycoo_structure_world_frame(),

    waypoints_x_coo_gps_frame(),
    waypoints_y_coo_gps_frame()

{
}


//Corpo FUnzioi diciarate nel file.h
//----> solo le funzioni dichhiarate nel file .h (se pubbliche) possono essere chiaamate dal main


void  StructureImpl::transformation_point_from_structure_to_world_frame()
{
    //Transformation structure centers from body frame (structure body frame) to World frame
    //The angle theta of structure orientation is set in main
    int size_v = xcoo_structure_frame.size();

    Eigen::Matrix3f T;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f structure_P_xcoo_vector_in_structure_frame(xcoo_structure_frame[size_v - 1],ycoo_structure_frame[size_v - 1], 1);

     T << cos(_theta), -sin(_theta), xcenter_structure_world_frame[0],
    sin(_theta), cos(_theta), ycenter_structure_world_frame[0],
    0, 0,1;

    coo_w = T *structure_P_xcoo_vector_in_structure_frame;

    xcoo_structure_world_frame.push_back(coo_w[0]);
    ycoo_structure_world_frame.push_back(coo_w[1]);

}


//Se la funzione non è definita nella classe le variabili devono essere passate
void StructureImpl::transformation_structure_center_from_body_to_world()
{
     //Transformation structure centers from body frame (structure body frame) to World frame
    //The angle theta of structure orientation is set in main
    int size_v = xcenter_structure_frame.size();
    Eigen::Matrix3f T;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f structure_center_vector_in_structure_frame(xcenter_structure_frame[size_v - 1],ycenter_structure_frame[size_v - 1], 1);

    // float structure_center_vector_in_structure_frame[3] = {structure -> xcenter_structure_frame[size_v - 1],structure -> ycenter_structure_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T
    T << cos(_theta), -sin(_theta), xcenter_structure_world_frame[0],
    sin(_theta), cos(_theta), ycenter_structure_world_frame[0],
    0, 0,1;

    coo_w = T*structure_center_vector_in_structure_frame;


    xcenter_structure_world_frame.push_back(coo_w[0]);
    ycenter_structure_world_frame.push_back(coo_w[1]);


}

void StructureImpl::transformation_GPS_point_from_GPS_to_world_frame()
{
    //Transformation structure centers from body frame (structure body frame) to World frame
    int size_v = waypoints_x_coo_gps_frame.size();

    Eigen::Matrix3f T;
    Eigen::Matrix3f T1;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f coo_w1(0.0,0.0,0.0);
    Eigen::Vector3f GPS_point_in_gps_frame(waypoints_x_coo_gps_frame[size_v - 1],waypoints_y_coo_gps_frame[size_v - 1], 1);

   //Prima trasformazione dal frame GPS al frame Panel (il frame GPS fa riferimento al frame dei pannelli in prima istanza)
    //La matrice di trasformazione viene definita come rotazione di angolo gamma e traslazione delta(asse x) e eta (asse y) rispetto frame pannelli
     T << cos(_GPS_gamma), -sin(_GPS_gamma), _GPS_delta,
    sin(_GPS_gamma), cos(_GPS_gamma), _GPS_eta,
    0, 0,1;

    coo_w = T *  GPS_point_in_gps_frame;

    //Seconda trasformazione da frame Panel (dove ho riportato i punti del GPS) al frame world
    T1 << cos(_theta), -sin(_theta), xcenter_structure_world_frame[0],
    sin(_theta), cos(_theta), ycenter_structure_world_frame[0],
    0, 0,1;

    coo_w1 = T1 * coo_w;
    //cout << coo_w1 << endl;
    waypoints_x_coo_world_frame.push_back(coo_w1[0]);
    waypoints_y_coo_world_frame.push_back(coo_w1[1]);


}

void StructureImpl::place_structure_start_end_point(float n_structure, float distance_between_structures,int configuration)
{
     bool even = false; //false indica i punti da sinistra veros destra , true da destra verso sinistra
                       // i punti vengono aggiunti nel vettore come P1(destra) --> P2(sinistra)
                       // P2(sinistra) < --- P1(destra)
                       //ecc

    if (configuration == 1)
    {
    /*      -----------
     *      ------
     *      ------
     *      ----------
     */
      bool  flag_structure_disposition_1 = true;
    }

    bool flag_structure_disposition_1 = false;

    //Place Panel point P1 and P2 before in Panel frame
    for (int i = 0; i <  n_structure; i++)
    {
        if (even == false)
        {
            //Aggiungo punti in vettore da sinistra verso destra  P1 ---> P2
             xcoo_structure_frame.push_back(xcenter_structure_frame[i] - (_length/2));
             ycoo_structure_frame.push_back(ycenter_structure_frame[i]);

            //Aggiungo Punti GPS in corriposndenza dei punti P1-->P2 nel frame structure ---> IL frame GPS è collocato in corripondenza del frame structure
            //quindi sul centro del pannello 1
             waypoints_x_coo_gps_frame.push_back(xcenter_structure_frame[i] - (_length/2));
             waypoints_y_coo_gps_frame.push_back(ycenter_structure_frame[i]);

             //Panel point trasformation to wolrd frame
             transformation_point_from_structure_to_world_frame();

            //GPS point trasformation to wolrd frame
             transformation_GPS_point_from_GPS_to_world_frame(); //--> trasfromarzione da GPS frame a structure frame poi da structure frame a world frame

             //P2
             xcoo_structure_frame.push_back(xcenter_structure_frame[i] + (_length/2));
             ycoo_structure_frame.push_back(ycenter_structure_frame[i]);

             //P2 in gps frame
             waypoints_x_coo_gps_frame.push_back(xcenter_structure_frame[i] + (_length/2));
             waypoints_y_coo_gps_frame.push_back(ycenter_structure_frame[i]);

             if (flag_structure_disposition_1 == true && i > 0)
             {
                 //Create New Configuration
                 /* P1 --------------- P2
                  * P2 ------- P1
                  * P1 ------- P2 <--- siamo qui
                  * P2 ------------- P1
                  */
                  //Change position of P2: P1 --> P2
                 xcoo_structure_frame[xcoo_structure_frame.size() - 1] = xcenter_structure_frame[i];
                 ycoo_structure_frame[ycoo_structure_frame.size() - 1] = ycenter_structure_frame[i];

                 waypoints_x_coo_gps_frame[waypoints_x_coo_gps_frame.size() -1] = xcenter_structure_frame[i];
                 waypoints_y_coo_gps_frame[waypoints_y_coo_gps_frame.size() -1] = ycenter_structure_frame[i];
                 flag_structure_disposition_1 = false;
             }

             even = true;

            // cout <<"P1 x structure "<< i+1<<"in structure frame: "<< xcoo_structure_frame[xcoo_structure_frame.size() - 2]<<endl;
            // cout <<"P1 y structure "<< i+1<<"in structure frame: "<< ycoo_structure_frame[xcoo_structure_frame.size() - 2]<<endl;
            // cout <<"P2 x structure "<< i+1<<"in structure frame: "<< xcoo_structure_frame[xcoo_structure_frame.size() - 1]<<endl;
            // cout <<"P2 y structure "<< i+1<<"in structure frame: "<< ycoo_structure_frame[xcoo_structure_frame.size() - 1]<<endl;

            //Transformation of P1 P2 from structure body frame to world frame
            transformation_point_from_structure_to_world_frame();

            transformation_GPS_point_from_GPS_to_world_frame();
        }
        else
        {
            //Aggiungo punti in vettore da destra  verso sinistra P2 <---- P1
            //P1
            // cout<<"xcenter_structure_frame[i]: "<< xcenter_structure_frame[i] << endl;
            // cout<<"ycenter_structure_frame[i]: "<< ycenter_structure_frame[i] << endl;
            xcoo_structure_frame.push_back(xcenter_structure_frame[i] + (_length/2));
            ycoo_structure_frame.push_back(ycenter_structure_frame[i]);

            //P1 in gps frame
             waypoints_x_coo_gps_frame.push_back(xcenter_structure_frame[i] + (_length/2));
             waypoints_y_coo_gps_frame.push_back(ycenter_structure_frame[i]);


             if (flag_structure_disposition_1 == true)
             {
                 //Create New Configuration
                 /* P1 --------------- P2
                  * P2 ------- P1 <--- siamo qui
                  * P1 ------- P2
                  * P2 ------------- P1
                  */
                  //Change position of P1 --> coincide con la metà del pannello, dove è collocato il frame
                 xcoo_structure_frame[xcoo_structure_frame.size() - 1] = xcenter_structure_frame[i];
                 ycoo_structure_frame[ycoo_structure_frame.size() - 1] = ycenter_structure_frame[i];

                 waypoints_x_coo_gps_frame[waypoints_x_coo_gps_frame.size() -1] = xcenter_structure_frame[i];
                 waypoints_y_coo_gps_frame[waypoints_y_coo_gps_frame.size() -1] = ycenter_structure_frame[i];

             }
             transformation_point_from_structure_to_world_frame();
             transformation_GPS_point_from_GPS_to_world_frame();

             //P2
            xcoo_structure_frame.push_back(xcenter_structure_frame[i] - (_length/2));
            ycoo_structure_frame.push_back(ycenter_structure_frame[i]);
            //P2 in gps frame
            waypoints_x_coo_gps_frame.push_back(xcenter_structure_frame[i] - (_length/2));
            waypoints_y_coo_gps_frame.push_back(ycenter_structure_frame[i]);

            transformation_point_from_structure_to_world_frame();
            transformation_GPS_point_from_GPS_to_world_frame();

            even = false;


            cout <<"P1 x structure "<< i+1<<"in structure frame: "<< xcoo_structure_frame[xcoo_structure_frame.size() - 2]<<endl;
            // cout <<"P1 y structure "<< i+1<<"in structure frame: "<< ycoo_structure_frame[xcoo_structure_frame.size() - 2]<<endl;
            // cout <<"P2 x structure "<< i+1<<"in structure frame: "<< xcoo_structure_frame[xcoo_structure_frame.size() - 1]<<endl;
            // cout <<"P2 y structure "<< i+1<<"in structure frame: "<< ycoo_structure_frame[xcoo_structure_frame.size() - 1]<<endl;
        }

    }

}

void StructureImpl::init(float n_structure, float distance_between_structures,int configuration)
{


    //Insert centers of the structure in the map
     xcenter_structure_world_frame.push_back( _structure_center_W[0]);
     ycenter_structure_world_frame.push_back( _structure_center_W[1]);


    //Define centers in structure frame coordinates --> i want structure align algong the same x coo and different y coo
    for (float i = 0; i < n_structure; i++)
    {
        xcenter_structure_frame.push_back(0.0);
        ycenter_structure_frame.push_back(- i * distance_between_structures); //Panels aligned along axe y in negative direction
        cout<< "x center structure "<< i + 1 <<"frame : " << ycenter_structure_frame[i] << endl;
        cout<< "y center structure "<< i + 1 <<"frame : " << ycenter_structure_frame[i] << endl;

        //Trasformation from structure frame to world frame.. The center of structure 1 in world frame is located in main
        transformation_structure_center_from_body_to_world();
    }
    //Place Point Start and end ( P1 and P2 ) for each structure
    place_structure_start_end_point(n_structure, distance_between_structures, configuration);


}

//Obtain GPS error from MAin
void StructureImpl::pass_to_class_GPS_error(float GPS_gamma, float GPS_eta, float GPS_delta)
{
     _GPS_delta = GPS_delta;
     _GPS_gamma = GPS_gamma;
     _GPS_eta = GPS_eta;


}

//Pass Variables from class StructureImpl to class Panel to main
vector<float> StructureImpl::obtain_waypoints_x_coo_gps_frame()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return waypoints_x_coo_gps_frame;
}

vector<float> StructureImpl::obtain_waypoints_y_coo_gps_frame()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return waypoints_y_coo_gps_frame;
}

vector<float> StructureImpl::obtain_waypoints_x_coo_world_frame()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return waypoints_x_coo_world_frame;
}

vector<float> StructureImpl::obtain_waypoints_y_coo_world_frame()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return waypoints_y_coo_world_frame;
}

vector<float> StructureImpl::obtain_xcoo_structure_world_frame()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return xcoo_structure_world_frame;
}

vector<float> StructureImpl::obtain_ycoo_structure_world_frame()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return ycoo_structure_world_frame;
}

float StructureImpl::obtain_theta()
{
    //Pass to file main vector waypoints_x_coo_gps_frame
    return _theta;
}

#endif
