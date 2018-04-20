#ifndef PROJECT_CURRENT_LISTENER_H
#define PROJECT_CURRENT_LISTENER_H

#include <stdio.h>
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <string>

#include "sensor_msgs/JointState.h"
#include "grasping/stop_table.h"

const std::string STOP_TOPIC = "allegroHand_0/stop_topic";
const std::string STOP_TABLE_TOPIC = "allegroHand_0/stop_table_topic";
const std::string CURRENT_LISTENER_TOPIC = "allegroHand_0/current_listener";
const std::string NEXT_STATE_TOPIC = "allegroHand_0/next_state";

class currentListener {

 public:

    currentListener();

    void stopTableCallback(const grasping::stop_table &msg);

    void stopCallback(const std_msgs::String::ConstPtr &msg);

    void currentListenerCallback(const sensor_msgs::JointState &msg);

 private:
  ros::NodeHandle nh;
  ros::Time tnow;
  ros::Time tstart;
  ros::Subscriber desired_grasp_type_sub;
  ros::Subscriber stop_sub;
  ros::Subscriber stop_table_sub;
  ros::Publisher next_state_pub;
  sensor_msgs::JointState current_joint_state;
    
};

#endif //PROJECT_CURRENT_LISTENER_H
