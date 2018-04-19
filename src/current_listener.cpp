#include <grasping/current_listener.h>

#define DOF_JOINTS 16


double velocity[DOF_JOINTS] = {0.0};
double dt;
int stop_table[16];
bool cond;
int back = 0;
int speedPer = 1;

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

//Simulated velocity parameter (keyboard)
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

//Simulated hand direction command (keyboard)
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

//Force sensor data
  if (condition.compare("thumb_up") == 0) {
    stop_table[15] = 1;
    back = 0;
  } 

  if (condition.compare("thumb_down") == 0) {
    stop_table[14] = 1;
    back = 0;
  } 

  if (condition.compare("index_up") == 0) {
    stop_table[3] = 1;
    back = 0;
  } 

  if (condition.compare("index_down") == 0) {
    stop_table[2] = 1;
    back = 0;
  } 

  if (condition.compare("middle_up") == 0) {
    stop_table[7] = 1;
    back = 0;
  } 

  if (condition.compare("middle_down") == 0) {
    stop_table[6] = 1;
    back = 0;
  } 

  if (condition.compare("little_up") == 0) {
    stop_table[11] = 1;
    back = 0;
  } 

  if (condition.compare("little_down") == 0) {
    stop_table[10] = 1;
    back = 0;
  } 


//Simulated force sensor data(keyboard)
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

