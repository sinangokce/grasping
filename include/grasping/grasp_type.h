#ifndef PROJECT_GRASP_TYPE_H
#define PROJECT_GRASP_TYPE_H

//#include "allegro_node.h"
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "handtracker/spper.h"
#include "glove_tekscan_ros_wrapper/LasaDataStreamWrapper.h"
#include "grasping/stop_table.h"
#include "grasping/sensor_data.h" //plot


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


// Functions for treating the data coming from the sensors
    void sensorDataCallback(const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg);

    void mapping(float *p, const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg);

    void plotSensorData(float sensor_data[]);

    void fillStopTable(int *p2, int sensor_stop[]);

    void publishStopTableToCurrentListenerNode(int stop_table[]);


// Functions to choose the grasp type
    void graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg);

    void compareString(std::string const &grasp_type);

    void moveToDesiredGraspType();

    

    


    void nextStateCallback(const sensor_msgs::JointState &msg);

    bool checkEquality(int array[]);

    float average(int a, int b);

    void initControllerxx();

    void doIt();
    
 protected:
    ros::NodeHandle nh;

    // Handles defined grasp types (std_msgs/String).
    ros::Subscriber grasp_type_sub;
    ros::Subscriber next_state_sub;
    ros::Subscriber tactile_sub;
    ros::Subscriber SpeedPer_sub;
    ros::Subscriber sensor_data_sub;

    ros::Publisher current_state_pub;
    ros::Publisher desired_state_pub;
    ros::Publisher stop_pub;
    ros::Publisher stop_table_pub;
    ros::Publisher sensor_data_pub;
    ros::Publisher wentback_pub;


    //boost::mutex *mutex;

    sensor_msgs::JointState desired_state;
    sensor_msgs::JointState current_state;
    handtracker::spper speedMsg;

    
    /*double home_pose[16] =
        {
            // Default (HOME) position (degrees), set at system start if
            // no 'initial_position.yaml' parameter is loaded.
            0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
            5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };*/

    double home_pose_2[16] =
        {
            // Default (HOME) position (degrees), set at system start if
            // no 'initial_position.yaml' parameter is loaded.
            0.0,  -0.17, 0.79, 0.79,  0.0, -0.17, 0.79, 0.79,
            0.09, -0.09, 0.87, 0.79,  1.05, 0.44, 0.26, 0.79
        }; 

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

  double desired_position[DOF_JOINTS] = {0.0};
};

#endif //PROJECT_GRASP_TYPE_H