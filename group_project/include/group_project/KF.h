#ifndef _KF_H_
#define _KF_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace Eigen;

class KF_Impl;

class KF
{
    private:
    KF_Impl *KF_implementation; 

    public:
    //Costruttore classe Panel
    KF();
    //Eigen::Vector2f target_point_P1, target_point_P2 --> Real states 
    //Eigen::Vector2f obs_points_P1, obs_points_P1 --> observed points required to estimate the line parameters 
    void Kalman_Filter_calculate( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2x, float obs_points_P2_y );
    void pass_to_class_initialization_matrices(Eigen::Matrix2f Px, Eigen::Matrix2f R);
    void Kalman_filter_initialization( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y);

    //Output --> estimated line paramters in world frame
    Eigen::Vector2f Obtain_Kalman_filter_estimated_state();
   Eigen::Vector2f Obtain_Kalman_filter_observation(); 

    
    ~KF();
};

#endif