#include <grasping/grasp_type.h>
#include <unistd.h>
#include <stdio.h>


#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)


//double velocity[DOF_JOINTS] = {0.0};
int joint[16];
int stop_table[16];
//int condinit;
float speed_Percentage=1;
float hand_Direction=0;
double desired_position[DOF_JOINTS] = {0.0};
double current_position[DOF_JOINTS] = {0.0};
//double previous_position[DOF_JOINTS] = {0.0};
double distance[DOF_JOINTS] = {0.0};

int desiredisgreater[16];
bool startclosing = true;

bool first_run;
bool separated;

AllegroNodeGraspController::AllegroNodeGraspController() {
         
  initControllerxx();

  grasp_type_sub = nh.subscribe("allegroHand_0/libsss_cmd", 1, &AllegroNodeGraspController::graspTypeControllerCallback, this);

  SpeedPer_sub = nh.subscribe("/lwr/speedPercentage", 10, &AllegroNodeGraspController::speedPerCallback, this);

  desired_state_pub = nh.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 1);

  sensor_data_sub = nh.subscribe("/LasaDataStream", 10, &AllegroNodeGraspController::sensorDataCallback, this);

  sensor_data_pub = nh.advertise<grasping::sensor_data>(SENSOR_DATA_TOPIC, 10); //plot

  //next_state_sub = nh.subscribe(NEXT_STATE_TOPIC, 1, &AllegroNodeGraspController::nextStateCallback, this);

  //current_state_pub = nh.advertise<sensor_msgs::JointState>(CURRENT_LISTENER_TOPIC, 1);

  //stop_pub = nh.advertise<std_msgs::String>(STOP_TOPIC, 1); 

  //stop_table_pub = nh.advertise<grasping::stop_table>(STOP_TABLE_TOPIC, 10); 

  //wentback_pub = nh.advertise<std_msgs::String>("allegroHand_0/libsss_cmd", 1);
}

void AllegroNodeGraspController::speedPerCallback(const handtracker::spper &msg) {
  speed_Percentage = msg.sPer;
  hand_Direction = msg.dir;
}

void AllegroNodeGraspController::sensorDataCallback(const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg) {

  for(int i = 0; i < 16; i++){
    stop_table[i] = 0;
  }  

  float sensor_data[8];
  float *p1;
  p1 = sensor_data;

  mapping(p1, msg);
  plotSensorData(sensor_data);

  int sensor_stop[8];
  int *p2;
  p2 = stop_table;

  for(int i = 0; i < 8; i++){     //THRESHOLD
    if(sensor_data[i] > 1) 
      sensor_stop[i] = 1;
    else  
      sensor_stop[i] = 0;
  }  

  fillStopTable(p2, sensor_stop);

}

void AllegroNodeGraspController::mapping(float *p, const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg) {

  //THUMB
  *p     = average((msg.ring2_f[10] + msg.ring2_f[11] + msg.pinky2_f[8] + msg.ring2_s[9] + msg.ring2_s[10] + msg.ring2_s[11]),  6);
  *(p+1) = average((msg.ring3_f[4]+msg.ring3_f[5]+msg.ring3_f[6] + msg.index1_s[9]+msg.index1_s[10]+msg.index1_s[11] + msg.middle3_s[7] + msg.ring3_s[0]+msg.ring3_s[1]+msg.ring3_s[2]+msg.ring3_s[4]+msg.ring3_s[5]), 12);

//INDEX
  *(p+2) = average((msg.middle1_f[0]+msg.middle1_f[1]+msg.middle1_f[2]+msg.middle1_f[3]+msg.middle1_f[4]+msg.middle1_f[5]+msg.middle1_f[6]+msg.index1_s[7]+msg.middle1_s[0]+msg.middle1_s[1]+msg.middle1_s[2]+msg.middle1_s[4]+msg.middle1_s[5]), 13);
  *(p+3) = average((msg.index2_f[0]+msg.index2_f[1]+msg.index2_f[2]+msg.index2_f[3]+msg.index2_s[0]+msg.index2_s[1]+msg.index2_s[2]), 7);

//MIDDLE
  *(p+4) = average((msg.ring1_f[0]+msg.ring1_f[1]+msg.ring1_f[2]+msg.ring1_f[3]+msg.ring1_f[4]+msg.ring1_f[5]+msg.ring1_f[6]+msg.ring1_s[0]+msg.ring1_s[1]+msg.ring1_s[2]+msg.ring1_s[4]+msg.ring1_s[5]), 12);
  *(p+5) = average((msg.middle1_f[14]+msg.middle1_f[15]+msg.middle2_f[0]+msg.middle2_f[1]+msg.middle2_f[2]+msg.middle2_f[3]+msg.middle1_s[10]+msg.middle1_s[11]+msg.middle1_s[13]+msg.middle1_s[14]+msg.middle1_s[15]+msg.middle2_s[0]+msg.middle2_s[1]+msg.middle2_s[2]), 14);

//RING
  *(p+6) = average((msg.ring1_f[14]+msg.ring1_f[15]+msg.pinky1_f[0]+msg.pinky1_f[1]+msg.pinky1_f[2]+msg.pinky1_f[3]+msg.pinky1_f[4]+msg.pinky1_f[5]+msg.pinky1_f[6]+msg.pinky1_f[12]+msg.ring1_s[7]+msg.ring1_s[10]+msg.ring1_s[11]+msg.ring1_s[13]+msg.ring1_s[14]+msg.ring1_s[15]+msg.pinky1_s[0]+msg.pinky1_s[1]+msg.pinky1_s[2]+msg.pinky1_s[4]+msg.pinky1_s[5]+msg.pinky1_s[8]), 22);
  *(p+7) = average((msg.ring2_f[0]+msg.ring2_f[1]+msg.ring2_f[2]+msg.ring2_f[3]+msg.ring2_s[0]+msg.ring2_s[1]+msg.ring2_s[2]+msg.ring2_s[3]), 8);
}

void AllegroNodeGraspController::plotSensorData(float sensor_data[]) {

  grasping::sensor_data sensor_data_msg;
  
  sensor_data_msg.upper_thumb  =  sensor_data[0];
  sensor_data_msg.lower_thumb  =  sensor_data[1];
  sensor_data_msg.upper_index  =  sensor_data[2];
  sensor_data_msg.lower_index  =  sensor_data[3];
  sensor_data_msg.upper_middle =  sensor_data[4];
  sensor_data_msg.lower_middle =  sensor_data[5];
  sensor_data_msg.upper_little =  sensor_data[6];
  sensor_data_msg.lower_little =  sensor_data[7];

  sensor_data_pub.publish(sensor_data_msg);

}

void AllegroNodeGraspController::fillStopTable(int *p2, int sensor_stop[]) {

  if(sensor_stop[0] == 1) {
    *(p2+15) = 1;
  }
  else if(sensor_stop[0] == 0) {
    *(p2+15) = 0;
  }

  if(sensor_stop[1] == 1) {
    *(p2+14) = 1;
    *(p2+13) = 1;
    *(p2+12) = 1;
  }
  else if(sensor_stop[1] == 0) {
    *(p2+14) = 0;
    *(p2+13) = 0;
    *(p2+12) = 0;
  }

  if(sensor_stop[2] == 1) {
    *(p2+3) = 1;
  }
  else if(sensor_stop[2] == 0) {
    *(p2+3) = 0;
  }

  if(sensor_stop[3] == 1) {
    *(p2+2) = 1;
    *(p2+1) = 1;
    *(p2+0) = 1;
  }
  else if(sensor_stop[3] == 0) {
    *(p2+2) = 0;
    *(p2+1) = 0;
    *(p2+0) = 0;
  }


  if(sensor_stop[4] == 1) {
    *(p2+7) = 1;
  }
  else if(sensor_stop[4] == 0) {
    *(p2+7) = 0;
  }

  if(sensor_stop[5] == 1) {
    *(p2+6) = 1;
    *(p2+5) = 1;
    *(p2+4) = 1;
  }
  else if(sensor_stop[5] == 0) {
    *(p2+6) = 0;
    *(p2+5) = 0;
    *(p2+4) = 0;
  }

  if(sensor_stop[6] == 1) {
    *(p2+11) = 1;
  }
  else if(sensor_stop[6] == 0){
    *(p2+11)= 0;
  }

  if(sensor_stop[7] == 1) {
    *(p2+10) = 1;
    *(p2+9) = 1;
    *(p2+8) = 1;
  }
  else if(sensor_stop[7] == 0){
    *(p2+10) = 0;
    *(p2+9) = 0;
    *(p2+8) = 0;
  }

}

void AllegroNodeGraspController::graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string grasp_type = msg->data;

  std::cout<< first_run << std::endl;

  compareString(grasp_type);

  if(first_run)
    moveToDesiredGraspType();
  else if(!first_run) {
    separateFingers();
    moveToDesiredGraspType();
  }

  first_run = false;

}

void AllegroNodeGraspController::compareString(std::string const &grasp_type) {
  std_msgs::String stop_msg;
  std::stringstream stop_ss;

  for(int i = 0; i < DOF_JOINTS; i++)
    desiredisgreater[i] = 0;

  if (grasp_type.compare("power") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = power[i];
  }

  else if (grasp_type.compare("thumb") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = thumb[i];
  }

  else if (grasp_type.compare("pinch") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = pinch[i];
  }

  else if (grasp_type.compare("lateral") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = lateral[i]; 
  }

  else if (grasp_type.compare("home") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = home_pose[i]; 
  }
}

void AllegroNodeGraspController::moveToDesiredGraspType() {

  for (int i = 0; i < DOF_JOINTS; i++) {
    distance[i] = desired_position[i] - current_state.position[i];
    current_state.velocity[i] = (distance[i]/30000);
    joint[i] = 0;
    stop_table[i] = 0;
  }

  startclosing = true;

  bool op = true;
  while(op){
    updateCurrentPosition();

    if(checkEquality(joint))
      break; 
    
    desired_state_pub.publish(current_state);
    usleep(10);
  }
}

void AllegroNodeGraspController::separateFingers(){

  joint[1] = 0;
  joint[5] = 0;
  //joint[9] = 0;
  //joint[14] = 0;

  distance[1] =  separated_posiiton[0] - current_state.position[1];
  distance[5] =  separated_posiiton[1] - current_state.position[5];
  //distance[9] =  separated_posiiton[2] - current_state.position[9];
  //distance[14] = separated_posiiton[3] - current_state.position[14];

  current_state.velocity[1] = (distance[1]/3000);
  current_state.velocity[5] = (distance[5]/3000);
  //current_state.velocity[9] = (distance[9]/3000);
  //current_state.velocity[14] = (distance[14]/3000);


  separated = false;
  
  while(!separated) {
    current_state.position[1] = current_state.position[1] + current_state.velocity[1]; 
    current_state.position[5] = current_state.position[5] + current_state.velocity[5]; 
    //current_state.position[9] = current_state.position[9] + current_state.velocity[9]; 
    //current_state.position[14] = current_state.position[14] + current_state.velocity[14]; 


    if (current_state.position[1] <= separated_posiiton[0]) 
      joint[1] = 1;
    if (current_state.position[5] <= separated_posiiton[1]) 
      joint[5] = 1;
    //if (current_state.position[9] <= separated_posiiton[2]) 
      //joint[9] = 1;
    //if (current_state.position[14] <= separated_posiiton[3]) 
      //joint[14] = 1;

    if(checkSeparate(joint)) {
      separated = true;
      break;
    }

    desired_state_pub.publish(current_state);
    usleep(100);

  }
}

void AllegroNodeGraspController::updateCurrentPosition() {
  
  for (int i = 0; i < (int)DOF_JOINTS; i++) {
    if (stop_table[i] == 1)
      current_state.velocity[i] = 0.0; 
  }

  for (int i = 0; i < (int)DOF_JOINTS; i++) 
    current_state.position[i] = current_state.position[i] + current_state.velocity[i]; 

  if(startclosing) {  //just to fill the table only once
    for (int i = 0; i < DOF_JOINTS; i++) {
      if (current_state.position[i] <= desired_position[i]) 
        desiredisgreater[i] = 1;
      else 
        desiredisgreater[i] = 0;
    }

    startclosing = false; //just to fill the table only once
  }

  for (int i = 0; i < DOF_JOINTS; i++) {
    if (desiredisgreater[i] == 1 && current_state.position[i] >= desired_position[i]) 
      joint[i] = 1;
    else if (desiredisgreater[i] == 0 && current_state.position[i] <= desired_position[i]) 
      joint[i] = 1;
  }  

}

bool AllegroNodeGraspController::checkSeparate(int array[]) {
  
  if(array[1] != 1 || array[5] != 1 /*|| array[9] != 1 || array[14] != 1*/)
    return false;
  
  return true;
}

bool AllegroNodeGraspController::checkEquality(int array[]) {
  for (int i = 0; i < DOF_JOINTS; i++) {
    if(array[i] != 1)
      return false;
  }
  return true;
}

float AllegroNodeGraspController::average(int sum, int number) {

  float avg;

  avg = sum / number;

  return avg;
}

void AllegroNodeGraspController::initControllerxx() {
  current_state.position.resize(DOF_JOINTS);
  current_state.velocity.resize(DOF_JOINTS);
  
  desired_state.position.resize(DOF_JOINTS);
  desired_state.velocity.resize(DOF_JOINTS);

  first_run = true;
}

void AllegroNodeGraspController::doIt() {
  ros::Rate rate(250.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}