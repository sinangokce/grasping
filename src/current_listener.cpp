//#include "allegro_node.h"
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

#define DOF_JOINTS 16

const std::string STOP_TOPIC = "allegroHand_0/stop_topic";
const std::string CURRENT_LISTENER_TOPIC = "allegroHand_0/current_listener";
const std::string NEXT_STATE_TOPIC = "allegroHand_0/next_state";

//const int DOF_JOINTS = 16;

double velocity[DOF_JOINTS] = {0.0};
double dt;
int stop_table[16];
bool cond;
int back = 0;
int speedPer = 1;

class currentListener
{
 public:
  currentListener();
  void stopCallback(const std_msgs::String::ConstPtr &msg);
  void currentListenerCallback(const sensor_msgs::JointState &msg);
  
 private:
  ros::NodeHandle nh;
  ros::Time tnow;
  ros::Time tstart;
  ros::Subscriber desired_grasp_type_sub;
  ros::Subscriber stop_sub;
  ros::Publisher next_state_pub;
  sensor_msgs::JointState current_joint_state;
};

currentListener::currentListener() {

  for (int i = 0; i < 16; i++) {
      stop_table[i] = 0;
    }

  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);


  stop_sub = nh.subscribe(STOP_TOPIC, 1, &currentListener::stopCallback, this);
  desired_grasp_type_sub = nh.subscribe(CURRENT_LISTENER_TOPIC, 1, &currentListener::currentListenerCallback, this);
  next_state_pub =nh.advertise<sensor_msgs::JointState>(NEXT_STATE_TOPIC, 1);
}

void currentListener::stopCallback(const std_msgs::String::ConstPtr &msg) {
  const std::string condition = msg->data;

  if (condition.compare("one") == 0) {
    speedPer = 1;
  }

  if (condition.compare("two") == 0) {
    speedPer = 2;
  }

  if (condition.compare("five") == 0) {
    speedPer = 5;
  }

  if (condition.compare("zero") == 0) {
    speedPer = 0;
  }


  if (condition.compare("false") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      stop_table[i] = 0;
    back = 0;
  }

  if (condition.compare("open") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      stop_table[i] = 0;
    back = 1;
  }  

  if (condition.compare("stop") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      stop_table[i] = 1;
    back = 0;
  } 

  if (condition.compare("close") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      stop_table[i] = 0;
    back = 0;
  }  

  if (condition.compare("little_tactile") == 0) {
    for (int i = 8; i < 12; i++) 
      stop_table[i] = 1;
    back = 0;
  }

  else if (condition.compare("middle_tactile") == 0) {
    for (int i = 4; i < 8; i++) 
      stop_table[i] = 1;
    back = 0;
  }

  else if (condition.compare("index_tactile") == 0) {
    for (int i = 0; i < 4; i++) 
      stop_table[i] = 1;
    back = 0;
  }

  else if (condition.compare("thumb_tactile") == 0) {
    for (int i = 12; i < 16; i++) 
      stop_table[i] = 1;
    back = 0;
  }
 
}

void currentListener::currentListenerCallback(const sensor_msgs::JointState &msg) {
  tnow = ros::Time::now();
  dt = 1e-9 * (tnow - tstart).nsec; 
  tstart = tnow;

  for (int i = 0; i < (int)DOF_JOINTS; i++) {
    current_joint_state.position[i] = msg.position[i]; 
    current_joint_state.velocity[i] = msg.velocity[i]; 
  }

  for (int i = 0; i < (int)DOF_JOINTS; i++) {
    if (stop_table[i] == 1)
      velocity[i] = 0.0; 
    else 
      velocity[i] = current_joint_state.velocity[i]*speedPer;
  }
  
  if (back == 1){
    for (int i = 0; i < (int)DOF_JOINTS; i++) {
      current_joint_state.position[i] = current_joint_state.position[i] - velocity[i] / dt; //0.001; //velocity[i]*dt;
    }
  }
  else {
    for (int i = 0; i < (int)DOF_JOINTS; i++) {
      current_joint_state.position[i] = current_joint_state.position[i] + velocity[i] / dt; //0.001; //velocity[i]*dt;
    }
  }

  next_state_pub.publish(current_joint_state);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_current_listener");
  currentListener current_listener;
  ros::Rate rate(50.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}