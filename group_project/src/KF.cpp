#ifndef _KF_SOURCE_
#define _KF_SOURCE_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include "group_project/pid.h"
#include "group_project/Structure.h"
#include "group_project/KF.h"
using namespace std;
using namespace Eigen;

class KF_Impl
{
    private:
    int K_count = 0;
    Eigen::Vector2f xh; //GPS line parameters initial state
    Eigen::Matrix2f _Px;
    Eigen::Matrix2f _R;
    Eigen::Vector2f X;
    Eigen::Vector2f X_old;
    Eigen::Vector2f y;
    Eigen::Matrix2f  A;
    Eigen::Matrix2f  C;
    Eigen::Matrix2f  K;
    Eigen::Matrix2f  Px_;
    Eigen::Vector2f  xh_old;
    Eigen::Vector2f yh_;
    Eigen::Vector2f obs;

    Eigen::Vector2f eigen_Px;
    //Estimation Vector
    Eigen::Vector2f xh_;
    float stdev;
    vector<float> c_obs_v;

    public:

    //Class constructor, prende i termini dati in input alla class PID
    KF_Impl();

    void Kalman_Filter_calculate( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y  );
    void pass_to_class_initialization_matrices( Eigen::Matrix2f Px, Eigen::Matrix2f R);
    void Kalman_filter_initialization( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y);

    Eigen::Vector2f Obtain_Kalman_filter_estimated_state();
    Eigen::Vector2f Obtain_Kalman_filter_observation();
    //Destructor
     ~KF_Impl()
    {

    }

};

//Class PanelImpl call from header
KF::KF()
{
    KF_implementation = new KF_Impl();
}

//Function dcall
void KF::Kalman_Filter_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y )
{
    return KF_implementation -> Kalman_Filter_calculate(target_point_P1_x,   target_point_P1_y,  target_point_P2_x,   target_point_P2_y,  obs_points_P1_x,  obs_points_P1_y , obs_points_P2_x,  obs_points_P2_y  );
}

void KF::pass_to_class_initialization_matrices(Eigen::Matrix2f Px, Eigen::Matrix2f R)
{
    return KF_implementation -> pass_to_class_initialization_matrices(Px, R);
}

void KF::Kalman_filter_initialization(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y)
{
    return KF_implementation -> Kalman_filter_initialization(target_point_P1_x, target_point_P1_y, target_point_P2_x,  target_point_P2_y, GPS_P1_x, GPS_P1_y, GPS_P2_x, GPS_P2_y);
}

Eigen::Vector2f KF::Obtain_Kalman_filter_estimated_state()
{
    return KF_implementation ->Obtain_Kalman_filter_estimated_state();
}

Eigen::Vector2f KF::Obtain_Kalman_filter_observation()
{
     return KF_implementation ->Obtain_Kalman_filter_observation();
}

//Il distruttore dealloca anche la classe Panel
KF::~KF()
{
    delete KF_implementation;
}

/* Implementation */

KF_Impl::KF_Impl():
//Definisco TUTTE le variabiliche verranno utilizzate nelle funzioni e definite in PanelImpl
//Le variabili passate con costruttore devono essere rinominate e passate al file cpp definendole in private
K_count(),
c_obs_v(),
stdev(),
xh_(),
X(),
y(),
A(),
C(),
K(),
_Px(),
_R(),
Px_(),
xh_old(),
xh(),
X_old(),
yh_(),
eigen_Px(),
obs()

{
}

void KF_Impl::Kalman_Filter_calculate( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y )
{
    //Evaluate a,b,c equation parameters given the the target points in GFs


      float a_target = (target_point_P2_y - target_point_P1_y)/(target_point_P2_x - target_point_P1_x);
      float b_target = 1;
      float c_target = ((-1*a_target * target_point_P1_x) + target_point_P1_y);



      //Evaluate a,b,c equation parameters given the the obs points in GF
      float a_obs = (obs_points_P2_y -obs_points_P1_y)/(obs_points_P2_x -obs_points_P1_x);
      float b_obs = 1;
      float c_obs = ((-1*a_obs * obs_points_P1_x) + obs_points_P1_y);

      cout<< "[KALMAN FILTER] a_target : "<< a_target << "     c_target: "<< c_target << endl;
      cout<< "[KALMAN FILTER] target point P1:" << target_point_P1_x << "," << target_point_P1_y << endl;
      cout<< "[KALMAN FILTER] obsv point P1:" << obs_points_P1_x << "," << obs_points_P1_y << endl;

      //cout<< "[KALMAN FILTER] OBSERVATIONS:  a_obs : "<< a_obs <<"    c_obs: "<< c_obs << endl;


      //Define sitution when the KF is not updated
      if (isnan(a_obs) == 1 || isnan(c_obs) == 1)
      {
          return;
      }

      if (K_count < 10)
     {
        if (c_obs < xh_[1] - 3.5 || c_obs > xh_[1] + 3.5)
           {
             return;
           }

     }
     else
      {
          if (a_obs < xh_[0] - 0.2 || a_obs >  xh_[0] + 0.2)
          {
             //Eliminate very wrong predictions
             return;
           }

            if (c_obs < xh_[1] - 2 || c_obs > xh_[1] + 2)
           {

               cout<< "KF.xh_[1]: " <<xh_[1]<<endl;
               return;
           }
      }
      //Evaluate mean and covariance of c_obs
      c_obs_v.push_back(c_obs);
      float sum = accumulate(c_obs_v.begin(), c_obs_v.end(), 0.0); //SOmma i valori nel vettore partendo da zero
      float mean = sum / c_obs_v.size();
      vector<double> diff(c_obs_v.size());
      transform(c_obs_v.begin(), c_obs_v.end(), diff.begin(),
               bind2nd(std::minus<double>(), mean));
      double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      stdev = sqrt(sq_sum / c_obs_v.size());

      //cout<< "[KALMAN FILTER]  mean: "<< mean << endl;
      //cout<< " [KALMAN FILTER]  stdev: "<< stdev << endl;
      if (abs(c_obs) - abs(xh_[1]) >  2.5* stdev and K_count > 40)
      {
          cout<<"SONO IN RETURN PER STD"<<endl;
            return;
      }
     //Define Matrices for aritmetic evaluations
      Eigen::Matrix2f res;
      Eigen::Matrix2f res1;
      Eigen::Matrix2f I;
      Eigen::Vector2f inov(0.0,0.0);

      I << 1.0 ,0.0,
      0.0, 1.0;

      //Update state iteration
      X << X_old[0], X_old[1];
      y << a_obs, c_obs;

    //Update state estimation and estimation covariance matrix with the previous values
     xh_ =  xh;

     Px_ = A * _Px * A;

//------------------- MEASUREMENT UPDATE:
//---------Kalamn Filter update K(t) = P(t|t-1) * C' * inv(C*P(t|t-1) * C' + R)
       res = Px_ * C;

       res1 = C * Px_* C + _R;

       res1 = res1.inverse().eval();

       K = res * res1;
//------------  Estimated  observation y(t|t-1) = C*X(t|t-1)
       yh_ = C * xh_;
// ----------- Measuremet residual innovation error y(t) - y(t|t-1)

       inov = y.transpose() - yh_.transpose();

//------------ A posteriori updated estimate of the current state x(t|t
       xh = xh_ + K * inov;


//------------ A posteriori updated state covariance matrix P(t|t) = (I - K(t)*C)

     _Px = (I - K* C) * Px_;

  //cout << "[KALMAN FILTER] Estimated state xh_:" << xh_ << endl;

     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(_Px);
     if (eigensolver.info() != Eigen::Success) abort();
     Eigen::Vector2f eigenvalues1 =  eigensolver.eigenvalues();

     eigen_Px = eigenvalues1;
    // cout<<" Eigen values of KF RGB Px: "<< eigenvalues1<< endl;

    //Insert observation inside vector obs to return to main
     obs << a_obs, c_obs;

     K_count = K_count + 1;
     xh_old =  xh;

}

//Funzione necessaria per inizializzare le matrici e sopratutto dare un valore iniziale allo stato da stimare
//relativo alla posizione della vela.
//Chiamare la funzione al momento del passaggio da una vela a quella sucessiva
void KF_Impl::Kalman_filter_initialization( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y)
{


      float a_target = (target_point_P2_y -target_point_P1_y)/(target_point_P2_x -target_point_P1_x);
      float b_target = 1;
      float c_target = ((-1*a_target * target_point_P1_x) + target_point_P1_y);



      //Evalaute line equation from GPS waypoints for initialization for each panels line
      float a_GPS = (GPS_P2_y -GPS_P1_y)/(GPS_P2_x - GPS_P1_x);
      float b_GPS = 1;
      float c_GPS = ((-1*a_GPS * GPS_P1_x) + GPS_P1_y);

    //Initilize state and matrices of the Kalman each time a new panel array is started
      X << a_target, c_target;
      X_old << a_target, c_target;
      //y << a_obs, c_obs;
      xh << a_GPS, c_GPS;

      A << 1.0, 0.0,
          0.0,1.0;
      C << 1.0, 0.0,
          0.0,1.0;
      K << 0.0, 0.0,
          0.0, 0.0;

      Px_ << 0.0, 0.0,
          0.0, 0.0;

      xh_old << 0.0, 0.0;

      xh_ <<  xh[0], xh[1];


      K_count = 0;

      cout << "[KALMAN FILTER] State Initialization with GPS line parameters: " << xh << endl;
}

//Obtain Initialization Matrices  from MAin
//Inizializzazione delle matrici di covarianza
//Chiamare la funzione subito dopo aver inizializzato la calsse
void KF_Impl::pass_to_class_initialization_matrices(Eigen::Matrix2f Px, Eigen::Matrix2f R)
{

     _Px = Px;
     _R = R;



}

// Kalman FIlter Output
Eigen::Vector2f KF_Impl::Obtain_Kalman_filter_estimated_state()
{
    return xh_;
}

 Eigen::Vector2f KF_Impl::Obtain_Kalman_filter_observation()
 {
     return obs;
 }

#endif
