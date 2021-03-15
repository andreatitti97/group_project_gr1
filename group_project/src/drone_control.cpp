#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SetModelState.h"
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include "group_project/pid.h"
#include "group_project/Structure.h"
#include "group_project/KF.h"

using namespace std;
#define N 2

/*L'algoritmo genera, partendo dalla distribuzione della struttura nell'ambiente di simulazione, una sucessione 
di waypoint "GPS", rototraslati rispetto alla disposizione veritiera della struttura in simulazione, la cui reale posizione ovviamente è conosciuta.

La struttura in riferimento, la quale non puo essere divulgata per motivi di privacy voluti dall'azienda con la quale collaboriamo
(Se doveste capirlo vi prego di tenervelo per voi),
è una struttura lineare disposta ad array.

I waypoints sono utilizzati dal drone per raggiungere il punto di start dopo il decollo, per inizializzre lo stato del filtro di Kalman
e per permettere il passaggio da una struttura lineare alla sucessiva.

Man mano che il drone avaza lungo la struttura, inizialmente seguendo i punti di waypoints fornit dall'utente,
inizia a ricevere osservazioni, pubblicate dall'algoritmo Python man mano che nuove feature vengono riconosciute.

Queste osservazioni, ottenute come punto nel frame del drone (punti che giacciono sulla retta calcolata dall'algoritmo in Py),
vengono date come input ad un filtro di Kalman, il quale genera una stima piu precisa dello stato, definito come i parametri m e q di una retta.

Un controllo predittivo viene utilizzato per permettere al drone di seguire la retta stimata.

L'algoritmp è basato per funzionare solo  con la telecamera RGB, solo con quella termica o entrambe. 

In seguito troverete commenti piu specifici per le varie sezioni del codice.

Se avete dubbi contattatemi pure a : luca.morando@edu.unige.it
*/

unsigned int sleep(unsigned int seconds);

//Struttura relativa alla gestione dei punti di waypoints
struct Waypoint_GPS
{
    float delta = 0.0; //Fattore di traslazione rispetto structure frame (locato nel cnetro struttura 1) su asse x
    float eta = 0.0;   ///Fattore di traslazione rispetto structure frame (locato nel cnetro struttura 1) su asse y
    float gamma = 0.0; //rotazione sistema di waypoint gps rispetto structure frame
    //Coordinata punto 1 (riferimento) del GPS. Da inizializzare in main con coordinata P1_x P1_y del primo struttura
    float GPS1_x = 0.0;
    float GPS1_y = 0.0;

    float error_from_GPS_line = 0.0;

    vector<float> waypoints_x_coo_world_frame;
    vector<float> waypoints_y_coo_world_frame;
    vector<float> waypoints_x_coo_gps_frame;
    vector<float> waypoints_y_coo_gps_frame;

    float GPS_P1_x = 0.0;
    float GPS_P1_y = 0.0;
    float GPS_P2_x = 0.0;
    float GPS_P2_y = 0.0;

    //Vector to initialize the Kalamn filter state with the equation of the line passing through the GPS
    //Waypoints
    vector<float> GPS_P1_waypoint; //Start P1 Waypoint
    vector<float> GPS_P2_waypoint; //End P2 waypoint
} waypoints;

//Struttura relativa alla gestione del drone.
struct Drone
{
    //Struct that link drone variables store in class drone outside of main
    //Drone Position, Orientation variables
    float drone_x, drone_y, drone_z;
    double drone_Yaw, drone_roll;
    float drone_ang_vel_z;
    float drone_lin_vel_x, drone_lin_vel_y, drone_lin_vel_z;

    geometry_msgs::Twist drone_vel_msg;

    float x_vel, y_vel, z_vel, x_angular_vel, y_angular_vel, z_angular_vel; //Desired Velocites
    float x_des, y_des, z_des;                                              //Desired positions
    float yaw_des;
    float yaw_err;

    float drone_x_b = 0.0;
    float drone_y_b = 0.0;

    //Drove Control points obtained from camera expressed in drone body frame
    float control_Thermo_point1_x, control_Thermo_point1_y;
    float control_Thermo_point2_x, control_Thermo_point2_y;
    float control_RGB_point1_x, control_RGB_point1_y;
    float control_RGB_point2_x, control_RGB_point2_y;

    float RGB_control_obs_P1_x_world = 0.0;
    float RGB_control_obs_P1_y_world = 0.0;
    float RGB_control_obs_P2_x_world = 0.0;
    float RGB_control_obs_P2_y_world = 0.0;

    float thermo_control_obs_P1_x_world = 0.0;
    float thermo_control_obs_P1_y_world = 0.0;
    float thermo_control_obs_P2_x_world = 0.0;
    float thermo_control_obs_P2_y_world = 0.0;

    //Control point D coordinates from GPS
    float control_x_coo = 0.0;
    float control_y_coo = 0.0;

    float x_target = 0.0;
    float y_target = 0.0;

    //KF counter
    int thermo_detection_count = 0;
    int RGB_detection_count = 0;

    Eigen::Vector2f xh_;  //KF estimated state
    Eigen::Vector2f xh_b; //KF estimated state in body frame
    Eigen::Vector2f obs;  //Obtain from class KF observation vector
    Eigen::Vector2f P1_B;
    Eigen::Vector2f P2_B;

    int image_control_count = 0;
    bool flagDroneOdom = false;
    bool flagDroneImu = false;
    bool flagDroneFix_vel = false;
    bool flagDroneThermoControlPoint1 = false;
    bool flagDroneThermoControlPoint2 = false;
    bool flagDroneRGBControlPoint1 = false;
    bool flagDroneRGBControlPoint2 = false;

} drone;

//Gestione della missione divisa in tre fasi: 1) start, 2) navigazione lungo la struttura 3) passsaggio ad una struttura sucessiva
struct Mission
{
    int state = 0;
    //Target : GPS o P ogni inizio struttura
    float x_target_P1 = 0.0;
    float y_target_P1 = 0.0;
    float x_target_P2 = 0.0;
    float y_target_P2 = 0.0;
    vector<float> P1_target;
    vector<float> P2_target;
    //Eigen::Vector2f Target;

    float x_target = 0.0;
    float y_target = 0.0;
    int target_point = 0.0;
    float cartesian_distance_err = 0.0;

    int count = 0;
    int navigation_iteration_count = 0;

    bool flag_end_point = false;
    bool flag_create_setpoint = false;
    bool structure_array_initialization = false;
    bool KF_Initialization = false;

} mission;

//Initialize Kalman FIlter
KF Kalman_Filter = KF();

nav_msgs::Odometry drone_odom;
sensor_msgs::Imu drone_imu;
geometry_msgs::Vector3Stamped drone_fix_vel;
geometry_msgs::Point drone_Thermo_control_point1;
geometry_msgs::Point drone_Thermo_control_point2;
geometry_msgs::Point drone_Thermo_control_point11;
geometry_msgs::Point drone_Thermo_control_point22;
geometry_msgs::Point drone_RGB_control_point1;
geometry_msgs::Point drone_RGB_control_point2;
std_msgs::Empty myMsg;

float check_x_b = 0.0;
float check_y_b = 0.0;
float check_x_w = 0.0;
float check_y_w = 0.0;

bool from_image = false;

void drone_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    drone_odom = *msg;
    drone.drone_x = drone_odom.pose.pose.position.x;
    drone.drone_y = drone_odom.pose.pose.position.y;
    drone.drone_z = drone_odom.pose.pose.position.z;

    // quaternion to RPY conversion
    tf::Quaternion q(drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y,
                     drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double drone_roll, drone_pitch, drone_yaw;
    m.getRPY(drone_roll, drone_pitch, drone_yaw);
    drone.drone_roll = drone_roll;
    drone.drone_Yaw = drone_yaw;
    // angular position
    drone.flagDroneOdom = true;
}

void drone_Imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    drone_imu = *msg;
    drone.drone_ang_vel_z = drone_imu.angular_velocity.z;
    drone.flagDroneImu = true;
}

void drone_fix_Vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    drone_fix_vel = *msg;
    drone.drone_lin_vel_x = drone_fix_vel.vector.x;
    drone.drone_lin_vel_y = drone_fix_vel.vector.y;
    drone.drone_lin_vel_z = drone_fix_vel.vector.z;
    drone.flagDroneFix_vel = true;
}

// OBTAIN CONTROL POINTS FROM THERMAL AND RGB IMAGES //
/*Queste callback permettono di ricevere i punti di osservazione pubblicati dagli algoritmi in python */
//#############  Desired Control POint 1 from Thermal images  ---> Usati per ottenere la retta esportata nel body vrame dall'image frame
void drone_Thermo_control_point1_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    drone_Thermo_control_point1 = *msg;

    drone.control_Thermo_point1_x = drone_Thermo_control_point1.x;
    drone.control_Thermo_point1_y = drone_Thermo_control_point1.y;

    drone.thermo_detection_count = 0;
    drone.flagDroneThermoControlPoint1 = true;
}

//Desired COntrol Point 2 from thermal Images
void drone_Thermo_control_point2_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    drone_Thermo_control_point2 = *msg;
    drone.control_Thermo_point2_x = drone_Thermo_control_point2.x;
    drone.control_Thermo_point2_y = drone_Thermo_control_point2.y;

    drone.thermo_detection_count = 0;
    drone.flagDroneThermoControlPoint2 = true;
}

// Desired Control POint 1 from RGB images ---> Usati per ottenere la retta esportata nel body vrame dall'image frame
void drone_RGB_control_point1_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    drone_RGB_control_point1 = *msg;
    drone.control_RGB_point1_x = drone_RGB_control_point1.x;
    drone.control_RGB_point1_y = drone_RGB_control_point1.y;
    cout << "RGB_point1_x = " << drone.control_RGB_point1_x << endl;
    cout << "RGB_point1_y = " << drone.control_RGB_point1_y << endl;
    drone.RGB_detection_count = 0;
    drone.flagDroneRGBControlPoint1 = true;
}

//Desired COntrol Point 2 from thermal Images
void drone_RGB_control_point2_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    drone_RGB_control_point2 = *msg;
    drone.control_RGB_point2_x = drone_RGB_control_point2.x;
    drone.control_RGB_point2_y = drone_RGB_control_point2.y;
    cout << "RGB_point2_x = " << drone.control_RGB_point2_x << endl;
    cout << "RGB_point2_y = " << drone.control_RGB_point2_y << endl;
    drone.RGB_detection_count = 0;
    drone.flagDroneRGBControlPoint2 = true;
}

/// ROTATION AND TRASFORMATION FUNCTION
void Rotation_GF_to_BF_des_pos(float x_pos, float y_pos, float alfa)
{
    check_x_b = x_pos * cos(alfa) + y_pos * sin(alfa) - cos(alfa) * drone.drone_x - sin(alfa) * drone.drone_y; //Checkpoint coordinate in drone body frame
    check_y_b = -x_pos * sin(alfa) + y_pos * cos(alfa) + sin(alfa) * drone.drone_x - cos(alfa) * drone.drone_y;
}

void Rotation_GF_to_BF_drone_pos(float alfa)
{
    drone.drone_x_b = drone.drone_x * cos(alfa) + drone.drone_y * sin(alfa) - drone.drone_x * cos(alfa) - drone.drone_y * sin(alfa); //Checkpoint coordinate in drone body frame
    drone.drone_y_b = -drone.drone_x * sin(alfa) + drone.drone_y * cos(alfa) + sin(alfa) * drone.drone_x - cos(alfa) * drone.drone_y;
}

void Rotation_BF_to_GF_des_pos(float x_pos, float y_pos, float alfa)
{
    check_x_w = x_pos * cos(alfa) - y_pos * sin(alfa) + drone.drone_x;
    check_y_w = x_pos * sin(alfa) + y_pos * cos(alfa) + drone.drone_y;
}

void saturation(float vel_x, float vel_y, float vel_z, float vel_yaw)
{
    if (vel_x > 1)
    {
        drone.drone_vel_msg.linear.x = 1;
    }
    else if (vel_x < -1)
    {
        drone.drone_vel_msg.linear.x = -1;
    }

    if (vel_y > 1)
    {
        drone.drone_vel_msg.linear.y = 1;
    }
    else if (vel_y < -1)
    {
        drone.drone_vel_msg.linear.y = -1;
    }
    if (vel_z > 1)
    {
        drone.drone_vel_msg.linear.z = 1;
    }
    else if (vel_z < -1)
    {
        drone.drone_vel_msg.linear.z = -1;
    }

    if (vel_yaw > 0.4)
    {
        drone.drone_vel_msg.angular.z = 0.4;
    }
    else if (vel_yaw < -0.4)
    {
        drone.drone_vel_msg.angular.z = -0.4;
    }
}

/*
Funzione che permette la navigazione e controllo del drone lungo la retta.
La retta puo essere:
1) quella tra due waypoints --> se bool from_image == false
2) quella stimaata da Kalman --> se bool from_image == true

Sostanzialmente un controllo PID viene applicato ad un punto che viene generato sulla retta da seguire.
Il punto non viene mai raggiunto, in quanto continuamente aggiornato.
La regola del parallelogramma tra un vettore parallelo alla retta e uno perpendicolare a quest'ultima 
(entrambi che partono dal centro del drone body frame), definice il vettore finale direzionato 
verso il punto di controllo predetto.
*/
void evaluate_control_point(bool from_image)
{
    //Equation line parameters Passing trhiug P1 and P2
    float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;

    float x_target_P1_body = 0.0;
    float y_target_P1_body = 0.0;
    float x_target_P2_body = 0.0;
    float y_target_P2_body = 0.0;

    //Rotate Point P1 in drone body frame
    Rotation_GF_to_BF_des_pos(waypoints.GPS_P1_waypoint[0], waypoints.GPS_P1_waypoint[1], drone.drone_Yaw);
    x_target_P1_body = check_x_b;
    y_target_P1_body = check_y_b;

    //Rotate Point P2 in drone body frame
    Rotation_GF_to_BF_des_pos(waypoints.GPS_P2_waypoint[0], waypoints.GPS_P2_waypoint[1], drone.drone_Yaw); // mission.y_target_P2
    x_target_P2_body = check_x_b;
    y_target_P2_body = check_y_b;

    if (from_image == false)
    {

        cout << "FOllowing GPS line!" << endl;

        //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
        a = ((y_target_P2_body - y_target_P1_body) / (x_target_P2_body - x_target_P1_body));
        c = ((-(a)*x_target_P1_body) + y_target_P1_body);
    }
    else
    {
        cout << "[IMAGE NAVIGATION] FOllowing Estimated line!" << endl;
        x_target_P1_body = drone.control_RGB_point1_x;
        y_target_P1_body = drone.control_RGB_point1_y;
        x_target_P2_body = drone.control_RGB_point2_x;
        y_target_P2_body = drone.control_RGB_point2_y;

        a = drone.xh_b[0];
        c = drone.xh_b[1];
    }

    //Find distance point line: point is the body Origin and line is the line r
    double e = (abs(c) / (sqrt(pow(a, 2) + pow(b, 2))));
    waypoints.error_from_GPS_line = e;

    //Define vector Vx starting from body frame origin and parallel to the line r
    double Vx[2] = {1 / a, -1 / b};

    double Kx = 0.3; //Coefficiente moltiplicativo del vettore parallelo alla retta r
    double Vx_norm[2] = {Kx * ((1 / (sqrt(pow(Vx[0], 2) + pow(Vx[1], 2)))) * Vx[0]), Kx * ((1 / (sqrt(pow(Vx[0], 2) + pow(Vx[1], 2)))) * Vx[1])};

    //cout<<"target_point: "<< target_point<<endl;
    float rot_Vx = 0.0;
    float rot_Vy = 0.0;

    if (mission.target_point == 1)
    {
        //punto di target è P1:
        if (x_target_P1_body >= 0 && Vx_norm[0] < 0)
        {

            rot_Vx = Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
            // cout << "sono qui" << endl;
        }
        else
        {
            Vx_norm[0] = Vx_norm[0];
            Vx_norm[1] = Vx_norm[1];
        }
    }
    else
    {
        if (x_target_P2_body >= 0 && Vx_norm[0] < 0)
        {
            rot_Vx = Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
            //  cout<< "sono quiiiiii"<< endl;
        }
        else
        {
            Vx_norm[0] = Vx_norm[0];
            Vx_norm[1] = Vx_norm[1];
        }
    }
    //Define vector Vy starting from body frame origin and perpendicular to line r
    //Il vettore Vy è moltiplicato pe run guadagno Ky e anche per l'errore dato dalla distanza punto retta e, la quale deve tendere a zero
    float Ky = 1; //Coefficiente moltiplicativo del vettore perp alla retta r
    double Vy_norm[2] = {0.0, 0.0};

    double Vy[2] = {1 / b, 1 / a};
    Vy_norm[0] = Ky * e * ((1 / (sqrt(pow(Vy[0], 2) + pow(Vy[1], 2)))) * Vy[0]);
    Vy_norm[1] = Ky * e * ((1 / (sqrt(pow(Vy[0], 2) + pow(Vy[1], 2)))) * Vy[1]);
    if (c > 0 && Vy_norm[1] < 0)
    {
        Vy_norm[1] = -1 * Vy_norm[1];
    }
    else if (c < 0 && Vy_norm[1] > 0)
    {
        Vy_norm[1] = -1 * Vy_norm[1];
    }
    //Find equation parameters of the line r1 passing trhoug vector Vx. Line r1 is parallel to line r but passing throung origin --> ha los tessto coefficiente angolare a della retta r
    float a1 = a;
    float b1 = -1;
    float c1 = 0;

    //Find equation parameters of the line r2 passing trhoug vector Vy. Line r2 is perpendicular  to line r but passing throung origin --> coeff angolare è
    float a2 = -1 / a; //Vy_norm[1]/Vy_norm[0]; era 1/a
    float b2 = -1;
    float c2 = 0;

    /*++++++++++++++Parallelogram rules. Find Vector V = Kx*Vx_norm + Ky*e*Vy_norm ***********/

    //Trovo retta r3 parallela a retta r2 ma passante per il punto B a cui tende vettore Vx
    float a3 = a2;
    float b3 = -1;
    float c3 = -a3 * Vx_norm[0] + Vx_norm[1];

    //Trovo retta r4 parallela a retta r1 ma passante per il punto C a cui tende vettore Vy
    float a4 = a1;
    float b4 = -1;
    float c4 = -a4 * Vy_norm[0] + Vy_norm[1];

    //Trovo punto di intersezioe D, al quale tende il vettore V. Il punto D è il punto di controllo
    float D[2] = {((b4 * c3 - b3 * c4) / (b3 * a4 - b4 * a3)), 0};
    D[1] = (-a3 * D[0] - c3) / b3;

    drone.control_x_coo = D[0]; // abs(D[0]);
    drone.control_y_coo = D[1];

    //Se le coordinate sono NAN alla prima iterazione le metto 0.0;
    if (drone.control_x_coo != drone.control_x_coo and drone.control_y_coo != drone.control_y_coo)
    {
        cout << "NAN" << endl;
        drone.control_x_coo = 0.0;
        drone.control_y_coo = 0.0;
    }

    drone.x_target = drone.control_x_coo;
    drone.y_target = drone.control_y_coo;
    cout<<"----->drone.x_target: " << drone.x_target << endl;
    cout<<"----->drone.y_target: " << drone.y_target << endl;
}

/*Il controlo è efettutao nel body frame del drone, quindi necessario
esportare la retta stimata nel frame del drone */
void KF_estimation_exportation_in_BF_for_control_point_generation()
{
    //Find two points on the line defined by the two paramter estimated
    float x1_w = 0.0;
    float y1_w = 0.0;
    float x2_w = 0.2;
    float y2_w = 0.0;

    float x1_b = 0.0;
    float y1_b = 0.0;
    float x2_b = 0.0;
    float y2_b = 0.0;

    float a_b = 0.0;
    float c_b = 0.0;

    //Use the RGB KF estimation
    y1_w = drone.xh_[0] * x1_w + drone.xh_[1];
    y2_w = drone.xh_[0] * x2_w + drone.xh_[1];
    //cout<<"[KF EXP FUNC] KF RGB estimation exported for control Point Evaluation" << endl;
    //cout<<"[KF EXP FUNC] y1_w: " <<y1_w << ", "<< "y2_w: " << y2_w << endl;

    //ROtate both points in body frame
    Rotation_GF_to_BF_des_pos(x1_w, y1_w, drone.drone_Yaw);
    x1_b = check_x_b;
    y1_b = check_y_b;

    Rotation_GF_to_BF_des_pos(x2_w, y2_w, drone.drone_Yaw);
    x2_b = check_x_b;
    y2_b = check_y_b;

    //Evaluate a,c parameters of the line in body frame
    a_b = ((y2_b - y1_b) / (x2_b - x1_b));
    c_b = ((-(a_b)*x1_b) + y1_b);

    drone.xh_b << a_b, c_b;
    //cout << "[KF EXP FUNC] ------------------____> drone.xh_b: " << drone.xh_b << endl;
}

/*Missione start: Da punto di decollo viene raggiunto il punto GPS 1, dal quale inizia la struttura da seguire */
void start(Structure *structure, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
    from_image = false;
    //In start mission.count = 0
    //Le coordinate di target sono le coordinate P1 del nuovo struttura, conosciute alla prima iterzione
    mission.x_target_P1 = structure->obtain_xcoo_structure_world_frame()[mission.count];
    mission.y_target_P1 = structure->obtain_ycoo_structure_world_frame()[mission.count];

    mission.x_target_P2 = structure->obtain_xcoo_structure_world_frame()[mission.count + 1];
    mission.y_target_P2 = structure->obtain_ycoo_structure_world_frame()[mission.count + 1];
    cout << "P1_x:" << mission.x_target_P1 << endl;
    cout << "P1_y:" << mission.y_target_P1 << endl;
    cout << "P2_x:" << mission.x_target_P2 << endl;
    cout << "P2_y:" << mission.y_target_P2 << endl;
    mission.P1_target.push_back(mission.x_target_P1);
    mission.P1_target.push_back(mission.y_target_P1);
    mission.P2_target.push_back(mission.x_target_P2);
    mission.P2_target.push_back(mission.y_target_P2);

    //Point Relative to GPS waypoints
    waypoints.GPS_P1_waypoint.push_back(waypoints.waypoints_x_coo_world_frame[mission.count]);
    waypoints.GPS_P1_waypoint.push_back(waypoints.waypoints_y_coo_world_frame[mission.count]);
    waypoints.GPS_P2_waypoint.push_back(waypoints.waypoints_x_coo_world_frame[mission.count + 1]);
    waypoints.GPS_P2_waypoint.push_back(waypoints.waypoints_y_coo_world_frame[mission.count + 1]);

    //evaluate_control_point(from_image);

    mission.x_target = structure->obtain_xcoo_structure_world_frame()[mission.count];
    mission.y_target = structure->obtain_ycoo_structure_world_frame()[mission.count];

    cout << "[START MISSION]--> Reaching P1 structure " << mission.count + 1 << " --> distance --> " << mission.cartesian_distance_err << endl;

    mission.cartesian_distance_err = sqrt(pow(mission.x_target - drone.drone_x, 2) + pow(mission.y_target - drone.drone_y, 2));
    drone.drone_vel_msg.angular.z = pid_yaw->position_control_knowing_velocity(drone.yaw_des, abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z);

    if (drone.drone_Yaw < 0)
    {
        drone.drone_vel_msg.angular.z = -1 * drone.drone_vel_msg.angular.z;
    }

    drone.yaw_err = drone.yaw_des - abs(drone.drone_Yaw);
    if (abs(drone.yaw_err) < 0.5)
    {
        Rotation_GF_to_BF_des_pos(mission.x_target, mission.y_target, drone.drone_Yaw);
        Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);

        drone.drone_vel_msg.linear.x = pid_x->calculate(check_x_b, drone.drone_x_b);
        drone.drone_vel_msg.linear.y = pid_y->calculate(check_y_b, drone.drone_y_b);
    }

    cout << "mission cartesian distance error: " << mission.cartesian_distance_err << endl;

    //Raggiunto Point P1 mi dirigo verso point P2
    if (mission.cartesian_distance_err < 0.3)
    {
        mission.flag_end_point = true; //Mi dirigo lungo il struttura verso punto P2 di end --> passo all'else sottostante
        mission.flag_create_setpoint = true;
        if (mission.count == 0)
        {
            drone.yaw_des = structure->obtain_theta();
        }

        drone.image_control_count = 100;   // inizializazzione valore grosso. --> Serve per aggiornare il filtro per un certo numero di iterazioni anche se nin sono state ricevute nuove osservazioni-- > utilizzal'ultima osservazione ricevuta
        mission.count = mission.count + 1; //--> Raggiungo punto P2 stesso struttura
        mission.target_point = 2;
        mission.KF_Initialization = true;
        mission.structure_array_initialization = true; //necessaria per la corretta inizializazzione Kalman Filter
        mission.state = 1;                             //Quando 1 mi muovo lungo il struttura tra punti 1 e 2
        cout << "Switch to NAVIGATION" << endl;
    }
}

/*Permette la navigazione lungo la struttura */
void navigation(Structure *structure, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
    //Reaching point P2 from point P1. P1 ---> P2
    cout << "Number of iteration in navigation: " << mission.navigation_iteration_count << endl;

    mission.x_target_P1 = structure->obtain_xcoo_structure_world_frame()[mission.count - 1];
    mission.y_target_P1 = structure->obtain_ycoo_structure_world_frame()[mission.count - 1];

    mission.x_target_P2 = structure->obtain_xcoo_structure_world_frame()[mission.count];
    mission.y_target_P2 = structure->obtain_ycoo_structure_world_frame()[mission.count];

    cout << "P1_x:" << mission.x_target_P1 << endl;
    cout << "P1_y:" << mission.y_target_P1 << endl;
    cout << "P2_x:" << mission.x_target_P2 << endl;
    cout << "P2_y:" << mission.y_target_P2 << endl;

    //Point to pass to Kalman FIlter class function

    mission.P1_target.push_back(mission.x_target_P1);
    mission.P1_target.push_back(mission.y_target_P1);
    mission.P2_target.push_back(mission.x_target_P2);
    mission.P2_target.push_back(mission.y_target_P2);

    //Point Relative to GPS waypoints
    waypoints.GPS_P1_x = waypoints.waypoints_x_coo_world_frame[mission.count - 1];
    waypoints.GPS_P1_y = waypoints.waypoints_y_coo_world_frame[mission.count - 1];
    waypoints.GPS_P2_x = waypoints.waypoints_x_coo_world_frame[mission.count];
    waypoints.GPS_P2_y = waypoints.waypoints_y_coo_world_frame[mission.count];

    //ROtate COntrol observation points obtained from thermo and RGB camera to world frame
    /*Rotation_BF_to_GF_des_pos(drone.control_Thermo_point1_x, drone.control_Thermo_point1_y, drone.drone_Yaw);
  
   drone.thermo_control_obs_P1_x_world = check_x_w;
   drone.thermo_control_obs_P1_y_world = check_y_w;
  
   Rotation_BF_to_GF_des_pos(drone.control_Thermo_point2_x, drone.control_Thermo_point2_y, drone.drone_Yaw);
   drone.thermo_control_obs_P2_x_world = check_x_w;
   drone.thermo_control_obs_P2_y_world = check_y_w; */

    Rotation_BF_to_GF_des_pos(drone.control_RGB_point1_x, drone.control_RGB_point1_y, drone.drone_Yaw);
    drone.RGB_control_obs_P1_x_world = check_x_w;
    drone.RGB_control_obs_P1_y_world = check_y_w;
    Rotation_BF_to_GF_des_pos(drone.control_RGB_point2_x, drone.control_RGB_point2_y, drone.drone_Yaw);
    drone.RGB_control_obs_P2_x_world = check_x_w;
    drone.RGB_control_obs_P2_y_world = check_y_w;

    //Initilize Kalman Filter
    if (mission.KF_Initialization == true)
    {
        Kalman_Filter.Kalman_filter_initialization(mission.x_target_P1, mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, waypoints.GPS_P1_x, waypoints.GPS_P1_y, waypoints.GPS_P2_x, waypoints.GPS_P2_y);
        mission.KF_Initialization = false;
    }

    //Update Kalman Filter estimation with thermal observation
    /* """"""" "
   COndizioni per entrare nel filtro:
   drone.flagDroneThermoControlPoint1 == true
   drone.flagDroneThermoControlPoint1 == true   ---> flag sono true quando la callback ha ricevuto le nuove osservazionidal topic
   mission.structure_array_initialization == false ---> il flag è true nelle prime 100 iteraioni del drone nella fase di navigazioni . Permette 
                                                    di muovere il drone lungo i waypoints GPS per iniziare a detectare la struttura e ricevere informazioni con cui aggiornare il filtro
    drone.image_control_count < 50 --> contatore che si aggiorna SOLO quando non sono ricevute ulteriori nuove osservazioni.
                                       Per 50 iterazion da quando i flag      flagDroneThermoControlPoint1 sono falsi il filtro viene aggiornato con le ultime informazioni disponibii.                                           

   */

    /*if(drone.flagDroneThermoControlPoint1 == true && drone.flagDroneThermoControlPoint2 ==true && mission.structure_array_initialization == false || drone.image_control_count < 50)
   {
      
       Kalman_Filter.Kalman_Filter_calculate( mission.x_target_P1,mission.y_target_P1, mission.x_target_P2, mission.y_target_P2,  drone.thermo_control_obs_P1_x_world ,  drone.thermo_control_obs_P1_y_world ,  drone.thermo_control_obs_P2_x_world ,  drone.thermo_control_obs_P2_y_world  );
       drone.xh_ = Kalman_Filter.Obtain_Kalman_filter_estimated_state();
       drone.obs = Kalman_Filter.Obtain_Kalman_filter_observation();
       cout << "[KALMAN FILTER THERMO] Observations : a  " << drone.obs[0] << " c: " << drone.obs[1] << endl;
       cout << "[KALMAN FILTER THERMO] Estimated states: a  " << drone.xh_[0] << " c: " << drone.xh_[1] << endl;
      from_image = true;
      if (drone.flagDroneThermoControlPoint1 == true && drone.flagDroneThermoControlPoint2== true &&  waypoints.error_from_GPS_line < 1.5) //drone.image_control_count: permette la guida guidata dal filtro anche quando non sono rileavate piu osservazioni,
                                                                                  //finche la condizione sull'errore dalla GPS line è rispettata 
      {
        //Resetto counter 
        drone.image_control_count = 0;
      }
       KF_estimation_exportation_in_BF_for_control_point_generation();
       
   }
   else
   {
       //Se non ci sono piu nuove osservazioni anche per oltre il numero concesso di osservazioni 
       //torno alla navigazione via GPS
       from_image = false; 
   }
   */

    //Update Kalamn FIlter Estimation with RGB observation if available
    if (drone.flagDroneRGBControlPoint1 == true && drone.flagDroneRGBControlPoint2 == true && mission.structure_array_initialization == false || drone.image_control_count < 30) //|| drone.image_control_count < 50
    {
        
        Kalman_Filter.Kalman_Filter_calculate(mission.x_target_P1, mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, drone.RGB_control_obs_P1_x_world, drone.RGB_control_obs_P1_y_world, drone.RGB_control_obs_P2_x_world, drone.RGB_control_obs_P2_y_world);
        cout << "RGB_P1_x = " << drone.RGB_control_obs_P1_x_world << endl;
        cout << "RGB_P1_y = " << drone.RGB_control_obs_P1_y_world << endl;
        cout << "RGB_P2_x = " << drone.RGB_control_obs_P2_x_world << endl;
        cout << "RGB_P2_y = " << drone.RGB_control_obs_P2_y_world << endl;
        drone.xh_ = Kalman_Filter.Obtain_Kalman_filter_estimated_state();
        drone.obs = Kalman_Filter.Obtain_Kalman_filter_observation();
        cout << "[KALMAN FILTER RGB] Observations : a  " << drone.obs[0] << " c: " << drone.obs[1] << endl;
        cout << "[KALMAN FILTER RGB] Estimated states: a  " << drone.xh_[0] << " c: " << drone.xh_[1] << endl;
        from_image = true;
        
        if (drone.flagDroneRGBControlPoint1 == true && drone.flagDroneRGBControlPoint2 == true && waypoints.error_from_GPS_line < 3) //3
        {
            //Resetto counter
            cout << "image control timer reset " << endl;
            drone.image_control_count = 0;
        }
        KF_estimation_exportation_in_BF_for_control_point_generation();
    }
    else
    {
        //Se non ci sono piu nuove osservazioni anche per oltre il numero concesso di osservazioni
        //torno alla navigazione via GPS
        cout << "from_image = false -> No image flag, high error, image control counter > 50" << endl;
        from_image = false; // TestAAA
    }

    //Evaluate control points to follow the GPS or estimated control line
    evaluate_control_point(from_image);

    drone.drone_vel_msg.angular.z = pid_yaw->position_control_knowing_velocity(drone.yaw_des, abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z); //ci andrebbe yaw
    if (drone.drone_Yaw < 0)
    {
        drone.drone_vel_msg.angular.z = -1 * drone.drone_vel_msg.angular.z;
    }

    // PID control on velocity defined on body frame
    // cout << "PID control based on target:" << drone.x_target << "," << drone.y_target << endl; 
    drone.drone_vel_msg.linear.x = pid_x->calculate(drone.x_target, drone.drone_x_b);
    drone.drone_vel_msg.linear.y = pid_y->calculate(drone.y_target, drone.drone_y_b);

    //Considero step

    //Permette a tutte le osservaziinidel filtri di essere inizializzate correttamente
    //finche non diventa false il flag il drone è guidato via Waypoint lungo la vela

    if (mission.navigation_iteration_count > 100) //cambiamento
    {
        mission.structure_array_initialization = false;
        cout << "Nav iteration count > 100 -> structure array init = false" << endl;
        // from_image = false;
    }

    if (from_image == true)
    {
        //FInche flag rimane true continuo ad incrementare counter.
        // verra resettato solo quando i flag callback sono true
        drone.image_control_count = drone.image_control_count + 1;
        cout << "from_image true so increment image control =" << drone.image_control_count << endl;
    }
    mission.navigation_iteration_count = mission.navigation_iteration_count + 1;

    mission.cartesian_distance_err = sqrt(pow(mission.x_target_P2 - drone.drone_x, 2) + pow(mission.y_target_P2 - drone.drone_y, 2)); //distanza dal raggiungere la fine del oannello
    if (mission.cartesian_distance_err < 1.5 or waypoints.error_from_GPS_line > 1.5)
    {
        from_image = false;
        cout << "from_image false since error GPS high" << endl;
    }
    //Passaggio alla mission.state 2--> Cambio di vela
    if (mission.cartesian_distance_err < 1.5)
    {
      
        from_image = false;
        //ROtating Yaw of 180 degree
        drone.yaw_des = drone.drone_Yaw + M_PI; //ROtate drone
        mission.state = 2;                      //Cambio di array
        mission.count = mission.count + 1;
        cout << "From Navigation mission.count incremented :" << mission.count << endl;
    }
}

/* Permette il passaggio alla struttura sucesiva 
Non penso che per voi sia interessante quetsa parte */
void jump_structure_array(Structure *structure, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
    cout << "Reaching NEW PANELS ARRAY" << endl;
    //Reaching point P3 from point P2. P2 ---> P3 --> Cambio struttura
    mission.x_target_P1 = structure->obtain_xcoo_structure_world_frame()[mission.count - 1]; //P1 diventa il precedente punto P2 di fine pannella
    mission.y_target_P1 = structure->obtain_ycoo_structure_world_frame()[mission.count - 1];

    mission.x_target_P2 = structure->obtain_xcoo_structure_world_frame()[mission.count];
    mission.y_target_P2 = structure->obtain_ycoo_structure_world_frame()[mission.count];

    mission.cartesian_distance_err = sqrt(pow(mission.x_target_P2 - drone.drone_x, 2) + pow(mission.y_target_P2 - drone.drone_y, 2));
    cout << "mission.cartesian_distance_err: " << mission.cartesian_distance_err << endl;
    //cout << "mission.x_target_P2:  " << mission.x_target_P2 << "mission.y_target_P2:  " << mission.y_target_P2 << endl;
    drone.drone_vel_msg.angular.z = pid_yaw->position_control_knowing_velocity(drone.yaw_des, abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z);
    cout << "YAW DES: " << drone.yaw_des << " drone.drone_Yaw: " << abs(drone.drone_Yaw) << endl;

    float yaw_err = drone.yaw_des - abs(drone.drone_Yaw);
    Rotation_GF_to_BF_des_pos(mission.x_target_P2, mission.y_target_P2, drone.drone_Yaw);
    Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);

    drone.drone_vel_msg.linear.x = pid_x->calculate(check_x_b, drone.drone_x_b);
    drone.drone_vel_msg.linear.y = pid_y->calculate(check_y_b, drone.drone_y_b);

    if (mission.cartesian_distance_err < 0.2 && yaw_err < 0.1)
    {
        mission.count = mission.count + 1; //Aggiorno target e waypoints con cui aggiornare anche KF
        drone.image_control_count = 100;   // inizializazzione valore grosso. --> Serve per aggiornare il filtro per un certo numero di iterazioni anche se nin sono state ricevute nuove osservazioni-- > utilizzal'ultima osservazione ricevuta
        mission.target_point = 2;
        mission.KF_Initialization = true;
        mission.structure_array_initialization = true; //necessaria per la corretta inizializazzione Kalman Filter
        mission.state = 1;                             // Incomincia nuova navigazione vela
    }
}

void publish_estimated_line(float a_estimated, float c_estimated)
{
    //Find two points on Inertial frame lies to the estimated line
    Eigen::Vector2f P1(0.0, 0.0);
    Eigen::Vector2f P2(1.0, 0.0);

    P1[1] = c_estimated; //y_P1 = a P1_x + c
    P2[1] = a_estimated * P2[0] + c_estimated;

    //Rotate in body/camera frame the points
    Rotation_GF_to_BF_des_pos(P1[0], P1[1], drone.drone_Yaw);
    P1[0] = check_x_b;
    P1[1] = check_y_b;
    Rotation_GF_to_BF_des_pos(P2[0], P2[1], drone.drone_Yaw);
    P2[0] = check_x_b;
    P2[1] = check_y_b;

    //Publish Point 1
    drone.P1_B = P1;
    drone.P2_B = P2;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Solar_fligth_control_sim");

    ros::NodeHandle nh;

    //Writing FIle  ---> Non necessario per voi salvare questi dati
    std::ofstream outFile("simulation_data/sim_data_complete/structure_x_point_in_world_frame.txt");
    std::ofstream outFile1("simulation_data/sim_data_complete/structure_y_point_in_world_frame.txt");
    std::ofstream outFile2("simulation_data/sim_data_complete/GPS_x_point_in_world_frame.txt");
    std::ofstream outFile3("simulation_data/sim_data_complete/GPS_y_point_in_world_frame.txt");

    bool Take_off = false;
    bool flag_even = false;
    bool flag_end_point = false;

    //PARAMETRS
    float n_structure = 4;
    float distance_between_structure = 2.2; //1.84;
    int configuration = 0;                  //1 se si vuole la configurazione con panneli a ferro di cavallo

    //Initialize GPS class
    waypoints.gamma = 0.5 * M_PI / 180; //Rotational error in rad
    waypoints.delta = 0.5;              //Traslational error along x respect structure frame located in structure 1
    waypoints.eta = 0.3;                //Traslational error along y

    //WayPoints waypoints_GPS(delta, eta, gamma);

    //Per le misure di ciasucna configurazione fare riferimento al file ardrone_testworld.world
    Eigen::Vector2f structure_center_W(6.37, 2); //6.37 , 2
    float theta = 0;                             //45.0 * M_PI/180;
    float size = 1.0;
    float length = 11; //11

    //Create a vector of structure for each structure
    Structure structure(structure_center_W, theta, size, length);

    //Obtain GPS error to define GPS waypoints for point P1 P2 of start and end
    structure.pass_to_class_GPS_error(waypoints.gamma, waypoints.eta, waypoints.delta);

    //Place structure centers and Start P1 and end P2 in map for each structure
    structure.init(n_structure, distance_between_structure, configuration);

    //Write structure points in world frame
    for (int i = 0; i < structure.obtain_xcoo_structure_world_frame().size(); i++)
    {
        outFile << structure.obtain_xcoo_structure_world_frame()[i] << "\n";
        outFile1 << structure.obtain_ycoo_structure_world_frame()[i] << "\n";
        cout << "structure " << i << " x coo in W: " << structure.obtain_xcoo_structure_world_frame()[i] << "," << structure.obtain_ycoo_structure_world_frame()[i] << "\n";
    }
    //Write GPS waypoints relative to structure points in world frame
    for (int i = 0; i < structure.obtain_waypoints_x_coo_world_frame().size(); i++)
    {
        outFile2 << structure.obtain_waypoints_x_coo_world_frame()[i] << "\n";
        outFile3 << structure.obtain_waypoints_y_coo_world_frame()[i] << "\n";

        cout << "structure " << i << " GPS waypoint in W: " << structure.obtain_waypoints_x_coo_world_frame()[i] << "," << structure.obtain_waypoints_y_coo_world_frame()[i] << "\n";
    }

    //Save in struct waypoints the GPS structure waypoints obtained from class structure
    for (int i = 0; i < structure.obtain_waypoints_x_coo_world_frame().size(); i++)
    {
        waypoints.waypoints_x_coo_world_frame.push_back(structure.obtain_waypoints_x_coo_world_frame()[i]);
        waypoints.waypoints_y_coo_world_frame.push_back(structure.obtain_waypoints_y_coo_world_frame()[i]);
        waypoints.waypoints_x_coo_gps_frame.push_back(structure.obtain_waypoints_x_coo_gps_frame()[i]);
        waypoints.waypoints_y_coo_gps_frame.push_back(structure.obtain_waypoints_y_coo_gps_frame()[i]);
    }

    //Define Gains Controller
    float Kp_z = 1.5;
    float Kd_z = 0.5;
    float Kp_yaw = 0.6;
    float Kd_yaw = 0.3;
    float Kp_x = 0.45;
    float Kp_y = 0.3; //0.2;
    float Kd_x = 0.01;
    float Kd_y = 0.02;
    float Ki_x = 0.15; //0.9; //0.1
    float Ki_y = 0.15; //0.55; //0.1

    double integralx(0);
    double integraly(0);
    float dt = 0.01;
    float time = 0.0;

    //Initialize PID controller for x, y, z and yaw direction (dt, max_value, min_value, Kp, Kd, Ki)
    //Output of PID are desired velocities

    PID pid_x = PID(dt, 3, -3, Kp_x, Kd_x, Ki_x);
    PID pid_y = PID(dt, 3, -3, Kp_y, Kd_y, Ki_y);

    PID pid_z = PID(dt, 3, -3, Kp_z, Kd_z, 0.01);
    PID pid_yaw = PID(dt, 1, -1, Kp_yaw, Kd_yaw, 0.01);

    Eigen::Matrix2f R;
    Eigen::Matrix2f Px;

    R << 0.2, 0.0,
        0.0, 0.25;
    Px << 4.0, 0.0,
        0.0, 4.0;

    Kalman_Filter.pass_to_class_initialization_matrices(Px, R);

    //General Variables
    float disturb = 0.0;
    //error on the drone body frame in PD controller
    float e_x = 0.0;
    float e_y = 0.0;
    float yaw_des = 0.0;
    float yaw_des_old = 0.0;
    float yaw_err = 0.0;
    float cartesian_distance_err = 0.0;  //distance between final structure point
    float cartesian_distance_err1 = 0.0; //distance between setpoints
    float drone_yaw_degree = 0.0;
    float distance_threshold = 0.2;
    int checkpoint = 1;

    //Publisher Topics
    ros::Publisher takeOff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher Land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Publisher vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher pub_P1_estimated = nh.advertise<geometry_msgs::Point>("/P1_estimated_control_point", 1);
    ros::Publisher pub_P2_estimated = nh.advertise<geometry_msgs::Point>("/P2_estimated_control_point", 1);
    //Publish drone position to camera in order to create idealistic gimbal
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient client_RGB = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Subscriber odom_drone_sub = nh.subscribe("/ground_truth/state", 50, drone_odom_callback);
    ros::Subscriber imu_drone_sub = nh.subscribe("/ardrone/imu", 5, drone_Imu_callback);
    ros::Subscriber vel_drone_sub = nh.subscribe("/fix_velocity", 1, drone_fix_Vel_callback);
    //Subscribe to the control points extracted from Thermal images
    ros::Subscriber control_point_1 = nh.subscribe("/Thermo_control_point_1", 1, drone_Thermo_control_point1_callback);
    ros::Subscriber control_point_2 = nh.subscribe("/Thermo_control_point_2", 1, drone_Thermo_control_point2_callback);

    //Subscribe to the second control points extracted from Thermal images
    //ros::Subscriber control_point_11 = nh.subscribe("/Thermo_control_point2_1", 1, drone_Thermo_control_point1_callback2);
    //ros::Subscriber control_point_22 = nh.subscribe("/Thermo_control_point2_2", 1, drone_Thermo_control_point2_callback2);

    //Subscribe to the control points extracted from RGB images
    ros::Subscriber control_RGB_point_1 = nh.subscribe("/RGB_control_point_1", 1, drone_RGB_control_point1_callback);
    ros::Subscriber control_RGB_point_2 = nh.subscribe("/RGB_control_point_2", 1, drone_RGB_control_point2_callback);

    //WRITE ON TXT simulation_data/sim_data_complete
    std::ofstream outFile4("simulation_data/sim_data_complete/des_x_vel.txt");
    std::ofstream outFile5("simulation_data/sim_data_complete/des_y_vel.txt");
    std::ofstream outFile6("simulation_data/sim_data_complete/des_z_vel.txt");
    std::ofstream outFile7("simulation_data/sim_data_complete/x_vel.txt");
    std::ofstream outFile8("simulation_data/sim_data_complete/y_vel.txt");
    std::ofstream outFile9("simulation_data/sim_data_complete/z_vel.txt");
    std::ofstream outFile10("simulation_data/sim_data_complete/drone_x_pos_saved.txt");
    std::ofstream outFile11("simulation_data/sim_data_complete/drone_y_pos_saved.txt");
    std::ofstream outFile13("simulation_data/sim_data_complete/y_error_VISION_LINE.txt");
    std::ofstream outFile14("simulation_data/sim_data_complete/roll.txt");
    std::ofstream outFile15("simulation_data/sim_data_complete/a_GPS.txt");
    std::ofstream outFile16("simulation_data/sim_data_complete/c_GPS.txt");
    std::ofstream outFile17("simulation_data/sim_data_complete/a_est.txt");
    std::ofstream outFile18("simulation_data/sim_data_complete/c_est.txt");
    std::ofstream outFile19("simulation_data/sim_data_complete/x_target.txt");
    std::ofstream outFile20("simulation_data/sim_data_complete/y_target.txt");
    std::ofstream outFile21("simulation_data/sim_data_complete/a_obs.txt");
    std::ofstream outFile22("simulation_data/sim_data_complete/c_obs.txt");

    //Da cancellare poi
    std::ofstream outFile23("simulation_data/sim_data_complete/KF_std_dev.txt");

    std::ofstream outFile24("simulation_data/sim_data_complete/a_RGB_est.txt");
    std::ofstream outFile25("simulation_data/sim_data_complete/c_RGB_est.txt");
    std::ofstream outFile26("simulation_data/sim_data_complete/KF_RGB_std_dev.txt");
    std::ofstream outFile27("simulation_data/sim_data_complete/a_RGB_obs.txt");
    std::ofstream outFile28("simulation_data/sim_data_complete/c_RGB_obs.txt");
    std::ofstream outFile29("simulation_data/sim_data_complete/Eigen_Px_KF_RGB.txt");
    std::ofstream outFile30("simulation_data/sim_data_complete/Eigen_Px_KF_thermal.txt");
    std::ofstream outFile31("simulation_data/sim_data_complete/drone_x_pos.txt");
    std::ofstream outFile32("simulation_data/sim_data_complete/drone_y_pos.txt");
    std::ofstream outFile33("simulation_data/sim_data_complete/drone_z_pos.txt");
    std::ofstream outFile34("simulation_data/sim_data_complete/error_from_GPS_line.txt");
    std::ofstream outFile35("simulation_data/sim_data_complete/error_from_vision_line.txt");

    mission.state = 0;
    //Counter
    int count = 0;
    int count_setpoint = 0;
    bool gps_waypoint = false;
    bool flag_create_setpoint = true;
    bool coming_back_to_GPS_path = false;
    int count_hovering = 0;

    ros::Rate r(20);
    while (nh.ok())
    {
        //Loop local variables
        float vel_x = 0.0;
        float vel_y = 0.0;
        float y = 0.0;

        drone.z_des = 5.0;
        //Desired_altitude

        if (Take_off == false)
        {
            takeOff.publish(myMsg);
            //Increasing altitude
            drone.drone_vel_msg.linear.z = pid_z.position_control_knowing_velocity(drone.z_des, drone.drone_z, 0, drone.drone_lin_vel_z);
            drone.yaw_des = atan2(structure.obtain_ycoo_structure_world_frame()[count + 1], structure.obtain_xcoo_structure_world_frame()[count + 1]); //Define yaw des to poiunt to the first P1 start structure
            drone.drone_vel_msg.angular.z = pid_yaw.position_control_knowing_velocity(drone.yaw_des, drone.drone_Yaw, 0, drone.drone_ang_vel_z);       //Yaw PID control
            //cout<<"drone.drone_vel_msg.linear.z: "<< drone.drone_vel_msg.linear.z<< endl;
            // publish the message
            vel.publish(drone.drone_vel_msg);

            drone.flagDroneOdom = false;
            drone.flagDroneImu = false;
            drone.flagDroneFix_vel = false;

            if (drone.drone_z < drone.z_des - 0.5)
            {
                cout << "Take_Off --> altitude: " << drone.drone_z << endl;
                ros::spinOnce();
                r.sleep();
                continue;
            }
            else
            {
                //Initialize distance error

                Take_off = true;
            }
        }

        // Maintain desired altitude and attitude
        drone.drone_vel_msg.linear.z = pid_z.position_control_knowing_velocity(drone.z_des, drone.drone_z, 0, drone.drone_lin_vel_z); //Kp_z * (drone.z_des - drone.drone_z) + Kd_z * (0 - drone.drone_lin_vel_z);
        drone_yaw_degree = drone.drone_Yaw * (180 / M_PI);

        switch (mission.state)
        {
        case 0:
            //Mission Initialization --> Reach point P1 structure after Take Off
            start(&structure, &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;

        case 1:
            //Reaching end of structure point P2 from point P1
            navigation(&structure, &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;

        case 2:
            //Cambio di vela
            jump_structure_array(&structure, &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;
        }

        //Publish Estimated Line from KF to add to the elaborated images taken from camera
        publish_estimated_line(drone.xh_[0], drone.xh_[1]);
        geometry_msgs::Point point1;
        point1.x = drone.P1_B[0];
        point1.y = drone.P1_B[1];

        geometry_msgs::Point point2;
        point2.x = drone.P2_B[0];
        point2.y = drone.P2_B[1];

        pub_P1_estimated.publish(point1);
        pub_P2_estimated.publish(point2);

        // publish velocity message
        saturation(drone.drone_vel_msg.linear.x, drone.drone_vel_msg.linear.y, drone.drone_vel_msg.linear.x, drone.drone_vel_msg.angular.z);
        vel.publish(drone.drone_vel_msg);

        /*
Nel caso abbiate bisogno di simulare una gimbal per incremntare la stabilità nella detection in Gazebo 
potete utilizzare questo servizio di gazebo che permette di 
spawnare la telecamera sempre nella posizione del drone,
Nel caso fosse necessario contattemi che vi spiego meglio.
*/
        //################## Define Bottom camera position relative to drone position-> idealistic Gimbal:
        gazebo_msgs::ModelState bottom_camera_pose;
        bottom_camera_pose.model_name = (std::string) "camera_box";
        bottom_camera_pose.reference_frame = (std::string) "world";
        bottom_camera_pose.pose.position.x = drone.drone_x;
        bottom_camera_pose.pose.position.y = drone.drone_y - 0.1;
        bottom_camera_pose.pose.position.z = drone.drone_z - 0.1;

        //Conversion of eulerian angle to quaternion for model state
        tf::Matrix3x3 obs_mat;
        obs_mat.setEulerYPR(drone.drone_Yaw, 0.0, 0.0);

        tf::Quaternion q_tf;
        obs_mat.getRotation(q_tf);
        bottom_camera_pose.pose.orientation.x = q_tf.getX();
        bottom_camera_pose.pose.orientation.y = q_tf.getY(); //Orientazione al fine di avere la camera sempre puntata verso il basso
        bottom_camera_pose.pose.orientation.z = q_tf.getZ();
        bottom_camera_pose.pose.orientation.w = q_tf.getW();

        //Call Service
        gazebo_msgs::SetModelState srv;
        srv.request.model_state = bottom_camera_pose;

        if (client.call(srv))
        {
            //ROS_INFO("camera box pose updated!!");
        }
        else
        {
            ROS_ERROR("Failed to update camera_box: Error msg:%s", srv.response.status_message.c_str());
        }
        //##########################################################

        //############################# Define RGB up camera position of the gimbal --> folow the drone position  ######################
        gazebo_msgs::ModelState bottom_camera_pose_RGB;
        bottom_camera_pose_RGB.model_name = (std::string) "camera_box_RGB";
        bottom_camera_pose_RGB.reference_frame = (std::string) "world";
        bottom_camera_pose_RGB.pose.position.x = drone.drone_x;
        bottom_camera_pose_RGB.pose.position.y = drone.drone_y;
        bottom_camera_pose_RGB.pose.position.z = drone.drone_z - 0.1;

        //Conversion of eulerian angle to quaternion for model state

        obs_mat.setEulerYPR(drone.drone_Yaw, 0.0, 0.0);

        obs_mat.getRotation(q_tf);
        bottom_camera_pose_RGB.pose.orientation.x = q_tf.getX();
        bottom_camera_pose_RGB.pose.orientation.y = q_tf.getY(); //Orientazione al fine di avere la camera sempre puntata verso il basso
        bottom_camera_pose_RGB.pose.orientation.z = q_tf.getZ();
        bottom_camera_pose_RGB.pose.orientation.w = q_tf.getW();

        //Call Service
        gazebo_msgs::SetModelState srv2;
        srv2.request.model_state = bottom_camera_pose_RGB;

        if (client_RGB.call(srv2))
        {
            //ROS_INFO("camera box pose updated!!");
        }
        else
        {
            ROS_ERROR("Failed to update camera_box: Error msg:%s", srv2.response.status_message.c_str());
        }

        //################################

        drone.flagDroneOdom = false;
        drone.flagDroneImu = false;
        drone.flagDroneFix_vel = false;
        drone.flagDroneThermoControlPoint1 = false;
        drone.flagDroneThermoControlPoint2 = false;
        drone.flagDroneRGBControlPoint1 = false;
        drone.flagDroneRGBControlPoint2 = false;

        time = time + dt;
        count_hovering = count_hovering + 1;

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
