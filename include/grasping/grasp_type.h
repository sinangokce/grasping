#ifndef PROJECT_GRASP_TYPE_H
#define PROJECT_GRASP_TYPE_H

#include <iostream>
#include <fstream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "handtracker/spper.h"
#include "glove_tekscan_ros_wrapper/LasaDataStreamWrapper.h"
#include "grasping/stop_table.h"
#include "grasping/sensor_data.h" //plot
#include <math.h> 
#include <stdlib.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <math.h>


#include <string>
#include <boost/thread/thread.hpp>

#include "sensor_msgs/JointState.h"


#define DOF_JOINTS 16

const std::string STOP_TOPIC = "allegroHand_0/stop_topic";
const std::string STOP_TABLE_TOPIC = "allegroHand_0/stop_table_topic";
const std::string SENSOR_DATA_TOPIC = "allegroHand_0/sensor_data_topic";//plot
const std::string CURRENT_LISTENER_TOPIC = "allegroHand_0/current_listener";
const std::string NEXT_STATE_TOPIC = "allegroHand_0/next_state";



class AllegroNodeGraspController {

 public:

    AllegroNodeGraspController();

    void speedPerCallback(const handtracker::spper &msg);
    void accCallback(const std_msgs::String::ConstPtr &msg);


// Functions for treating the data coming from the sensors
    void sensorDataCallback(const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg);

    void mapping(float *p, const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg);

    void plotSensorData(float sensor_data[]);

    void fillStopTable(int *p2, int sensor_stop[]);

// Functions to choose the grasp type
    void graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg);

    void compareString(std::string const &grasp_type);
    void getCurrentPosition(double jointAngles[16]);
    void potentialField();
    void getJointAnglesThumb(Eigen::Matrix<long double, 3, 1> pos_th/*, Eigen::Matrix<long double, 3, 1> pos_ind*/);
    void getJointAnglesIndex(Eigen::Matrix<long double, 3, 1> pos_index/*, Eigen::Matrix<long double, 3, 1> pos_ind*/);


    void smoothPositionControlling(double final_position[]);

    double sigmoidFunction(double initial_position,double distance, /*double velocity,*/ double time_sample);

    //void sampling(double ptrSamples[], int rows, int col);

    void moveToDesiredGraspType();

    void moveToDesiredGraspType2();

    //void openHand();

    

    //void scaleSamplesBetween0andPi(std::vector< std::vector<double> >samples);

    //void sinusoidalVelocity(std::vector< std::vector<double> >scaledSamples);

    void separateFingers();

    void separateSmart();

    void separateSmartly();

    void findVectors(double *ptrVect_a, double *ptrVect_height);

    double euclideanNorm(double vector[]);

    double semiPerimeter(double norm_a, double norm_b, double norm_c); 
       
    double triangleSurface(double s, double norm_a, double norm_b, double norm_c);

    //double triangleHeight(double area, double norm_b); 

    //double angleBetweenAandB(double norm_a, double norm_b, double norm_c);
   

    void updateCurrentPosition();


    //void nextStateCallback(const sensor_msgs::JointState &msg);
    //bool checkSeparate(int array[]);

    bool checkEquality(int array[]);

    float average(int a, int b);

    void initControllerxx();

    void doIt();
    
 protected:
    ros::NodeHandle nh;

    // Handles defined grasp types (std_msgs/String).
    ros::Subscriber grasp_type_sub;
    ros::Subscriber SpeedPer_sub;
    ros::Subscriber sensor_data_sub;
    ros::Subscriber acc_sub;

    ros::Publisher desired_state_pub;
    //ros::Publisher stop_pub;
    //ros::Publisher stop_table_pub;
    ros::Publisher sensor_data_pub;
    //ros::Publisher wentback_pub;


    //boost::mutex *mutex;

    sensor_msgs::JointState desired_state;
    sensor_msgs::JointState current_state;
    handtracker::spper speedMsg;

    ros::Time tnow;
    ros::Time tstart;
    ros::Time tnow_squared;
    ros::Time tstart_squared;

    
    /*double home_pose[16] =
        {
            // Default (HOME) position (degrees), set at system start if
            // no 'initial_position.yaml' parameter is loaded.
            0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
            5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };*/

    double home_pose[16] =
        {
            // Default (HOME) position (degrees), set at system start if
            // no 'initial_position.yaml' parameter is loaded.
            0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0
        }; 

    double power[16] =
        {
            0.19, 0.76, 0.91, 1.31,  0.0, 0.76, 0.91, 1.31,
           -0.19, 0.76, 0.91, 1.31, 1.40, 0.43, 0.92, 0.51
        }; 
       

    /*double power[16] =
        {
            -0.10, 0.76, 0.91, 0.0,  0.0, 0.76, 0.91, 1.31,
           -0.19, 0.76, 0.91, 1.31, 1.28, 0.43, 0.96, 0.0
        }; */


    double lateral[16] =
        {
           -0.06, 1.60, 1.60, 0.0,  0.0,  1.60, 1.60, 0.0,
            0.0,  1.60, 1.60, 0.0,  0.37, 0.29, 1.50, 0.0
        };  

    double pinch[16] =
        {
            0.0, 0.83, 0.89, 0.90, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0 , 0.0,  1.35, 0.0, 0.76, 0.29
        };       

    /*double pinch[16] =
        {
            0.0, 0.83, 0.89, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0 , 0.0,  1.35, 0.0, 0.90, 0.0
        };*/

    double thumb[16] =
        {
            0.0, 0.78, 1.09, 0.88,  -0.19, 0.82, 1.09, 0.80,
            0.0, 0.0,  0.0,  0.0,    1.35, 0.23, 0.70, 0.55                             
        }; 

    /*/double separated_posiiton[4] =
        {
           0.60, 0.60, 0.60, 0.30                    
        };*/     

  double desired_position[DOF_JOINTS] = {0.0};
};

#endif //PROJECT_GRASP_TYPE_H